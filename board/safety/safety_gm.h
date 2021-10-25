// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

const int GM_MAX_STEER = 300;
const int GM_MAX_RT_DELTA = 128;          // max delta torque allowed for real time checks
const uint32_t GM_RT_INTERVAL = 250000;    // 250ms between real time checks
const int GM_MAX_RATE_UP = 7;
const int GM_MAX_RATE_DOWN = 17;
const int GM_DRIVER_TORQUE_ALLOWANCE = 50;
const int GM_DRIVER_TORQUE_FACTOR = 4;
const int GM_MAX_GAS = 3072;
const int GM_MAX_REGEN = 1404;
const int GM_MAX_BRAKE = 350;
const uint32_t GM_LKAS_MIN_INTERVAL = 20000;    // 20ms minimum between LKAS frames
const int GM_GAS_INTERCEPTOR_THRESHOLD = 458;  // (610 + 306.25) / 2ratio between offset and gain from dbc file
#define GM_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks

const CanMsg GM_TX_MSGS[] = {{384, 0, 4}, {1033, 0, 7}, {1034, 0, 7}, {715, 0, 8}, {880, 0, 6}, {512, 0, 6},  // pt bus
                             {161, 1, 7}, {774, 1, 8}, {776, 1, 7}, {784, 1, 2},   // obs bus
                             {789, 2, 5},  // ch bus
                             {0x104c006c, 3, 3}, {0x10400060, 3, 5}};  // gmlan

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
AddrCheckStruct gm_addr_checks[] = {
  {.msg = {{388, 0, 8, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{842, 0, 5, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{481, 0, 7, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{241, 0, 6, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{417, 0, 7, .expected_timestep = 100000U}, { 0 }, { 0 }}},
  {.msg = {{513, 0, 6, .expected_timestep = 100000U}, { 0 }, { 0 }}}, //pedal inbound (enforce length in case of overlap)
  //{.msg = {{384, 0, 4, .expected_timestep = 100000U}, { 0 }, { 0 }}}, // Object and or chassis bus includes 384 with different size
};
#define GM_RX_CHECK_LEN (sizeof(gm_addr_checks) / sizeof(gm_addr_checks[0]))
addr_checks gm_rx_checks = {gm_addr_checks, GM_RX_CHECK_LEN};

//LKAS vars for ensuring in-order, correct timing

enum gm_lkas_state {gm_never,gm_recent,gm_past};

enum gm_lkas_state gm_lkas_ptbus_state = gm_never;



int gm_camera_bus = 2; // 2 is c2 default. 1 for obd

uint32_t gm_init_ts = 0; //

int gm_lkas_last_rc = -1; // Last rolling counter
uint32_t gm_lkas_last_ts = 0; // TS of last LKAS frame


//TODO: Make sure it is possible to RX before relay flips. More pervasive changes will be required if not

bool gm_camera_on_pt = true; // Block tx while camera is still on PT bus. Assume true.
bool gm_bad_cam_traffic = false; // Unexpected traffic on cam bus means radar or chassis
bool gm_enable_fwd = false; // All conditions are clear to enable forwarding!
int gm_good_lkas_cnt = 0; // Number of valid LKAS frames on cam bus up to limit


// TODO: Reminder: Handle remap of chassis and radar in OP

static int gm_next_rc(int current_rc) {
  //Should be faster than modulo
  int ret = 0;
  if (current_rc <= 2) {
    ret = current_rc + 1;
  }
  return ret;
}

static bool gm_verify_lkas(CAN_FIFOMailBox_TypeDef *to_check) {
  //Try to make sure message 384 is actually an LKAS message
  int is_correct = true;

  int len = GET_LEN(to_check);
  if (len != 4) {
    puts("gm_verify_lkas: Frame is wrong size!");
    is_correct = false;
  }
  else {
    int rolling_counter = GET_BYTE(to_check, 0) >> 4;
    if (rolling_counter < 0 || rolling_counter > 3) {
      is_correct = false;
      puts("gm_verify_lkas: Rolling counter out of range");
    }
    else {
      int desired_torque = ((GET_BYTE(to_check, 0) & 0x7U) << 8) + GET_BYTE(to_check, 1);
      if (!max_limit_check(desired_torque, GM_MAX_STEER + 200, -(GM_MAX_STEER + 200))) {
        puts("gm_verify_lkas: Torque out of range");
        is_correct = false;
      }
    }
  }
  return is_correct;
}




static int gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, &gm_rx_checks, NULL, NULL, NULL);

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);
    int len = GET_LEN(to_push);


    if (gm_camera_on_pt) {
      if (gm_lkas_ptbus_state == gm_recent) {
        // Check to see if has been long enough since the last lkas on PT to assume relay opened
        // Stock inactive frames are every 100ms - so 5 missed frames
        uint32_t ts = microsecond_timer_get();
        uint32_t elapsed = get_ts_elapsed(ts, gm_lkas_last_ts);
        if (elapsed > (500 * 1000)) {
          puts("gm_rx_hook: LKAS disappeared from PT, setting past and off PT");
          gm_lkas_ptbus_state = gm_past;
          gm_camera_on_pt = false;
        }
      }
      else if (gm_lkas_ptbus_state == gm_never) {
        // If it has been 2 seconds since startup and we have seen no LKAS on PT bus,
        // it is safe to assume we will not and we can enable TX
        uint32_t ts = microsecond_timer_get();
        uint32_t elapsed = get_ts_elapsed(ts, gm_init_ts);
        if (elapsed > (2 * 1000 * 1000)) {
          puts("gm_rx_hook: No LKAS on PT bus since startup for over 10 second - clearing flag");
          gm_camera_on_pt = false;
        }
      }
    }

    // LKAS messages should only be received on the PT bus if the relay is closed
    // Either it hasn't opened yet, or it faulted
    if (addr == 384) {
      puts("gm_rx_hook: LKAS frame on PT bus");
      //TODO: Do we need to run all this if in Dashcam mode?
      gm_enable_fwd = false;
      gm_camera_on_pt = true;
      gm_lkas_ptbus_state = gm_recent;
      uint32_t ts = microsecond_timer_get();
      gm_lkas_last_ts = ts;
      gm_lkas_last_rc = GET_BYTE(to_push, 0) >> 4;


      if ((gm_lkas_ptbus_state == gm_past) || gm_camera_on_pt == false) {
        puts("gm_rx_hook: unexpected LKAS frame on PT bus - relay malfunction?");
        relay_malfunction_set();
        // We will allow forwarding to resume if camera disappears from PT
        // Note on picky EPS chances are good LKAS faulted
        // Reset good frame counter
        gm_good_lkas_cnt = 0;
      }
    }

    // Pedal Interceptor
    if (addr == 0x201 && len == 6) {
      gas_interceptor_detected = 1;
      int gas_interceptor = GM_GET_INTERCEPTOR(to_push);
      gas_pressed = gas_interceptor > GM_GAS_INTERCEPTOR_THRESHOLD;
      gas_interceptor_prev = gas_interceptor;
    }

    if (addr == 388) {
      int torque_driver_new = ((GET_BYTE(to_push, 6) & 0x7) << 8) | GET_BYTE(to_push, 7);
      torque_driver_new = to_signed(torque_driver_new, 11);
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // sample speed, really only care if car is moving or not
    // rear left wheel speed
    if (addr == 842) {
      vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
    }

    //TODO: Swap activation to when cc is off!!
    // ACC steering wheel buttons
    if (addr == 481) {
      int button = (GET_BYTE(to_push, 5) & 0x70) >> 4;
      switch (button) {
        case 2:  // resume
        case 3:  // set
          controls_allowed = 1;
          break;
        case 6:  // cancel
          if (!gas_interceptor_detected) {
            controls_allowed = 0;
          }
          break;
        default:
          break;  // any other button is irrelevant
      }
    }

    // speed > 0
    if (addr == 241) {
      // Brake pedal's potentiometer returns near-zero reading
      // even when pedal is not pressed
      brake_pressed = GET_BYTE(to_push, 1) >= 10;
    }

    if (addr == 417) {
      gas_pressed = GET_BYTE(to_push, 6) != 0;
    }

    // exit controls on regen paddle
    if (addr == 189) {
      bool regen = GET_BYTE(to_push, 0) & 0x20;
      if (regen) {
        controls_allowed = 0;
      }
    }

    // Check if ASCM or LKA camera are online
    // on powertrain bus.
    // 384 = ASCMLKASteeringCmd
    // 715 = ASCMGasRegenCmd
    generic_rx_checks(((addr == 384) || (addr == 715)));
  }
  return valid;
}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int gm_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, GM_TX_MSGS, sizeof(GM_TX_MSGS)/sizeof(GM_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  int pedal_pressed = brake_pressed_prev && vehicle_moving;
  bool unsafe_allow_gas = unsafe_mode & UNSAFE_DISABLE_DISENGAGE_ON_GAS;
  if (!unsafe_allow_gas) {
    pedal_pressed = pedal_pressed || gas_pressed_prev;
  }
  bool current_controls_allowed = controls_allowed && !pedal_pressed;

  // GAS: Interceptor safety check
  if (addr == 0x200) {
    if (!current_controls_allowed) {
      if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
        puts("gas safety check failed");
        tx = 0;
      }
    }
  }

  // // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // // and the the latching controls_allowed flag is True
  // int pedal_pressed = gm_gas_prev || (gm_brake_prev && gm_moving);
  // bool current_controls_allowed = controls_allowed && !pedal_pressed;

  // BRAKE: safety check
  if (addr == 789) {
    int brake = ((GET_BYTE(to_send, 0) & 0xFU) << 8) + GET_BYTE(to_send, 1);
    brake = (0x1000 - brake) & 0xFFF;
    if (!current_controls_allowed) {
      if (brake != 0) {
        tx = 0;
      }
    }
    if (brake > GM_MAX_BRAKE) {
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 384) {
    if (gm_camera_on_pt != true) {
      tx = 0; //No LKAS from OP if camera is on PT bus
    }
    else {
      int rolling_counter = GET_BYTE(to_send, 0) >> 4;
      int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1);
      uint32_t ts = microsecond_timer_get();
      bool violation = 0;
      desired_torque = to_signed(desired_torque, 11);

      if (current_controls_allowed) {

        // *** global torque limit check ***
        violation |= max_limit_check(desired_torque, GM_MAX_STEER, -GM_MAX_STEER);

        // *** torque rate limit check ***
        violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
          GM_MAX_STEER, GM_MAX_RATE_UP, GM_MAX_RATE_DOWN,
          GM_DRIVER_TORQUE_ALLOWANCE, GM_DRIVER_TORQUE_FACTOR);

        // used next time
        desired_torque_last = desired_torque;

        // *** torque real time rate limit check ***
        violation |= rt_rate_limit_check(desired_torque, rt_torque_last, GM_MAX_RT_DELTA);

        // every RT_INTERVAL set the new limits
        uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
        if (ts_elapsed > GM_RT_INTERVAL) {
          rt_torque_last = desired_torque;
          ts_last = ts;
        }
      }

      // no torque if controls is not allowed
      if (!current_controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !current_controls_allowed) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }

      //TODO: Maybe should be checked at the moment the frame is sent via CAN - rcv interrupt could maybe prevent sending??
      if (tx == 1) {
        uint32_t lkas_elapsed = get_ts_elapsed(ts, gm_lkas_last_ts);
        int expected_lkas_rc = gm_next_rc(gm_lkas_last_rc);    //(gm_lkas_last_rc + 1) % 4;
        //If less than 20ms have passed since last LKAS message or the rolling counter value isn't correct it is a violation
        //TODO: The interval may need some fine tuning - testing the tolerance of the PSCM / send lag
        if (lkas_elapsed < GM_LKAS_MIN_INTERVAL || rolling_counter != expected_lkas_rc) {
          tx = 0;
        }
        else {
          //otherwise, save values
          gm_lkas_last_rc = rolling_counter;
          gm_lkas_last_ts = ts;
        }
      }

    }
  }

  // GAS/REGEN: safety check
  if (addr == 715) {
    int gas_regen = ((GET_BYTE(to_send, 2) & 0x7FU) << 5) + ((GET_BYTE(to_send, 3) & 0xF8U) >> 3);
    // Disabled message is !engaged with gas
    // value that corresponds to max regen.
    if (!current_controls_allowed) {
      bool apply = GET_BYTE(to_send, 0) & 1U;
      if (apply || (gas_regen != GM_MAX_REGEN)) {
        tx = 0;
      }
    }
    if (gas_regen > GM_MAX_GAS) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static const addr_checks* gm_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  gm_init_ts = microsecond_timer_get();

  if (car_harness_status == HARNESS_STATUS_NC) {
    puts("gm_init: No harness attached, assuming OBD or Giraffe");
    //OBD harness and older pandas use bus 1 and no relay
    gm_camera_bus = 1;
    gm_camera_on_pt = false;
  }

  return &gm_rx_checks;
}

static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;

  // TODO: Simplify logic!

  //Being careful to only forward when safe
  if (gm_enable_fwd && !gm_camera_on_pt && !gm_bad_cam_traffic) {
    if (bus_num == 0) {
      bus_fwd = gm_camera_bus; // PT Bus -> FFC
    }
    else if (bus_num == gm_camera_bus) {
      //Note: trusting that init completes before this
      int addr = GET_ADDR(to_fwd);
      //FFC -> PT Bus, block LKAS
      if (addr != 384) {
        bus_fwd = 0;
      }
      //TODO: long term - allow forwarding of stock camera and block op when op is disengaged
    }
  }
  else { //no forward OR pt on cam bus OR bad cam traffic
    if (bus_num == gm_camera_bus && !gm_camera_on_pt && !gm_bad_cam_traffic) {
      //camera is not on PT bus and we haven't seen anything bad on camera bus
      // That is, only when conditions are correct for forwarding, but forwarding ins't enabled
      int addr = GET_ADDR(to_fwd);

      if (addr == 384) {
        // Verify the LKAS frame. If it has 384 but doesn't have correct format permanently block forwarding
        if (gm_verify_lkas(to_fwd)) {
          gm_good_lkas_cnt++;
        }
        else {
          puts("gm_fwd_hook: Non-LKAS Frame ID 384 seen on cam bus, permabanning forwarding!");
          gm_good_lkas_cnt = 0;
          gm_bad_cam_traffic = true;
          gm_enable_fwd = false;
        }

        //If we have seen 9 valid LKAS frames on the cam bus, AND it is not on the PT, enable forwarding
        if (gm_good_lkas_cnt >= 9) {
          puts("gm_fwd_hook: 9 good LKAS frames on cam bus, conditions good, enabling forwarding!");
          gm_enable_fwd = true;
          bus_fwd = 0;
        }
      }
    }
  }

  return bus_fwd;
}



const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = gm_fwd_hook,
};
