// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

// TODO: Handle relay close - block OP first, wait for RC to line up, close relay

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
const uint32_t GM_LKAS_SETTLE_WINDOW = 10 * 1000 * 1000; // Time after which we assume LKAS state is settled
const uint32_t GM_LKAS_RELAY_WINDOW = 500 * 1000; // Time after last LKAS to assume LKAS gone
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
  {.msg = {{513, 0, 6, .expected_timestep = 100000U}, { 0 }, { 0 }}} //pedal inbound (enforce length in case of overlap)
};
#define GM_RX_CHECK_LEN (sizeof(gm_addr_checks) / sizeof(gm_addr_checks[0]))
#define GM_NEXT_RC ((gm_lkas_last_rc < 3) ? (gm_lkas_last_rc + 1) : 0)
addr_checks gm_rx_checks = {gm_addr_checks, GM_RX_CHECK_LEN};

//LKAS vars for ensuring in-order, correct timing and safe forwarding

// set in init
int gm_camera_bus = 2; // 2 is c2 default. 1 for obd
bool gm_has_relay = true; // If there is a relay (harness) present
uint32_t gm_init_ts = 0; // TS of init
// set in rx / tx
uint32_t gm_lkas_pt_last_ts = 0; // TS of last LKAS frame seen on PT
bool gm_relay_open_requested = false; // We must enable the relay ourselves so we can capture stock camera RC
bool gm_camera_off_pt = false; // Block tx while camera is still on PT bus
uint32_t gm_lkas_last_ts = 0; // TS of last LKAS frame sent to EPS
int gm_lkas_last_rc = -1; // Last rolling counter
// set in fwd
bool gm_fwd_enable = false; // All conditions are clear to enable forwarding!
bool gm_fwd_block = false; // No camera on cam bus
int gm_good_lkas_cnt = 0; // Number of valid LKAS frames on cam bus up to limit


static int gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, &gm_rx_checks, NULL, NULL, NULL);

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);

    if (!gm_camera_off_pt && !gm_fwd_block) {
      uint32_t ts = microsecond_timer_get();

      if (gm_lkas_pt_last_ts == 0) {
        uint32_t elapsed = get_ts_elapsed(ts, gm_init_ts);
        if (elapsed > GM_LKAS_SETTLE_WINDOW) {
          puts("gm_rx_hook: Assuming no LKAS on PT bus\n");
          gm_camera_off_pt = true;
        }
      }
      else {
        uint32_t elapsed = get_ts_elapsed(ts, gm_lkas_pt_last_ts);
        if (elapsed > GM_LKAS_RELAY_WINDOW) {
          puts("gm_rx_hook: LKAS gone from PT bus (relay opened)\n");
          gm_camera_off_pt = true;
        }
      }
    }

    // LKAS on PT means relay not open or no relay + bad wiring
    if (addr == 384 && !gm_fwd_block) {
      if (!gm_has_relay) {
        puts("gm_rx_hook: LKAS on PT without relay = bad wiring\n");
        gm_camera_off_pt = false;
        gm_fwd_block = true;
        gm_fwd_enable = false;
      }
      else {
        gm_camera_off_pt = false;
        gm_fwd_enable = false;
        uint32_t ts = microsecond_timer_get();
        gm_lkas_last_ts = ts;
        gm_lkas_pt_last_ts = ts;
        gm_lkas_last_rc = GET_BYTE(to_push, 0) >> 4;

        if (!gm_relay_open_requested) {
          puts("gm_rx_hook: Requesting relay open");
          //Copied from main.c
          set_intercept_relay(true);
          heartbeat_counter = 0U;
          heartbeat_lost = false;
          //
          gm_relay_open_requested = true;
        }
      }
    }

    // Pedal Interceptor
    if (addr == 0x201 && GET_LEN(to_push) == 6) {
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
    generic_rx_checks((addr == 384) || (addr == 715));
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
        puts("gas safety check failed\n");
        tx = 0;
      }
    }
  }

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
    if (!gm_camera_off_pt) {
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

      //TODO: Refactor as violation
      if (tx == 1) {
        uint32_t lkas_elapsed = get_ts_elapsed(ts, gm_lkas_last_ts);
        int expected_lkas_rc = GM_NEXT_RC;    //(gm_lkas_last_rc + 1) % 4;
        //If less than 20ms have passed since last LKAS message or the rolling counter value isn't correct it is a violation
        //TODO: The interval may need some fine tuning - testing the tolerance of the PSCM / send lag
        if (lkas_elapsed < GM_LKAS_MIN_INTERVAL || rolling_counter != expected_lkas_rc) {
          tx = 0;
          puts("### DROPPING LKAS. RC: ");
          putui(rolling_counter);
          puts(", Expected: ");
          putui(expected_lkas_rc);
          puts(", Elapsed since last: ");
          putui(lkas_elapsed);
          puts("\n");
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
  //puts("gm_init: Called\n");
  // Need to re-init in case of car off and on (harness always has power)
  gm_camera_bus = 2;
  gm_lkas_last_rc = -1;
  gm_has_relay = true;
  gm_relay_open_requested = false;
  gm_camera_off_pt = false;
  gm_fwd_block = false;
  gm_fwd_enable = false; // All conditions are clear to enable forwarding!
  gm_good_lkas_cnt = 0; // Number of valid LKAS frames on cam bus up to limit
  gm_lkas_last_ts = 0;
  gm_lkas_pt_last_ts = 0;
  gm_init_ts = microsecond_timer_get();

  if (car_harness_status == HARNESS_STATUS_NC) {
    //puts("gm_init: No harness attached, assuming OBD or Giraffe\n");
    //OBD harness and older pandas use bus 1 and no relay
    gm_has_relay = false;
    gm_camera_bus = 1;
  }

  return &gm_rx_checks;
}

static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  if (!gm_fwd_block) {
    if (gm_fwd_enable) {
      if (bus_num == 0) {
        bus_fwd = gm_camera_bus; // PT Bus -> FFC
      }
      else if (bus_num == gm_camera_bus) {
        if (GET_ADDR(to_fwd) != 384) {
          bus_fwd = 0;
        }
      }
    }
    else {
      if (gm_camera_off_pt && bus_num == gm_camera_bus && GET_ADDR(to_fwd) == 384) {
        if (GET_LEN(to_fwd) == 4) {
          gm_good_lkas_cnt++;
          if (gm_good_lkas_cnt > 9) {
            //puts("gm_fwd_hook: 10 good LKAS frames on cam bus, conditions good, enabling forwarding!\n");
            gm_fwd_enable = true;
          }
        }
        else {
          //puts("gm_fwd_hook: Non-LKAS Frame ID 384 seen on cam bus, permabanning forwarding!\n");
          gm_fwd_block = true;
          gm_fwd_enable = false;
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
