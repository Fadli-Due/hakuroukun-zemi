// bcd_automode.ino
// Autonomous-mode firmware for the Hakuroukun ride-on cleaning robot,
// running BCD (Boustrophedon Cell Decomposition) coverage path planning.
// Target: Arduino Mega 2560 flashed to /dev/arduino.
//
// ─── SERIAL PROTOCOL (115200 baud) ──────────────────────────────────────────
//   Request  (from hakuroukun_communication_node.py, 12 bytes on the wire):
//     "0" + D + SSS + AAA + "00" + "\r\n"
//        D   = 1 char, '0' = FORWARD, '1' = REVERSE
//        SSS = 3-digit steering pot target (000-999)
//        AAA = 3-digit accel    pot target (000-999)
//        Leading '0' and trailing "00" are protocol padding.
//        "\r\n" is the message terminator (Python appends it explicitly).
//   Reply    (variable length, ends with '\n'):
//     ST + D + SSS + AAA + ",pm_st=" + N + ",pm_ac=" + N
//        ST  = '0' OK, '1' malformed (length != 10), '2' watchdog was firing
//              before this message arrived (pedal was released; now recovered)
//        pm_st / pm_ac = current pot readings, for online diagnostics.
//   The receive side uses readStringUntil('\n') + trim() to strip '\r'.
//   This is the newer contract added in hakuroukun_communication_node.py
//   (Jul 2026 revision) — it replaces the older 2-ms-gap-based read used
//   by Tai's motor_control.ino, which was prone to partial-read failures
//   under load.
//
// ─── CALIBRATION (from keyboardmode.ino, 2026-07-07) ────────────────────────
//   Steering neutral   PM_ST_N   = 400
//   Steering right lim PM_ST_LIMR= 200  -> min pot = 200
//   Steering left  lim PM_ST_LIML= 290  -> max pot = 690
//   Accel neutral      PM_AC_N   = 235  (pedal fully released)
//   Accel press   lim  PM_AC_LIMU= 500  -> max pot = 735
//   Accel release lim  PM_AC_LIMD=  20  -> min pot = 215
//   These values were tuned by driving the robot manually with keyboard
//   teleop and observing pot readings at the mechanical stops. Do NOT edit
//   without re-running the manual calibration procedure.
//
//   HEADS-UP: the rad->pot conversion in hakuroukun_communication_node must
//   also match this calibration (PM_ST_N=400 and the asymmetric envelope).
//   Verify c_cw/c_ccw / slope constants in that node before an experiment.
//
// ─── SAFETY BEHAVIORS (new in BCD firmware; not in Tai's motor_control) ─────
//   1. Startup hold: motors are driven to neutral for STARTUP_HOLD_MS
//      before serial parsing begins. Prevents lurch if a ROS message
//      arrives while the pot control loop hasn't converged.
//   2. Serial watchdog: if no valid message arrives for WATCHDOG_MS, the
//      accel target is forced to neutral (pedal released). Steering target
//      is held at its last value — snapping steering to zero mid-turn
//      could destabilize the vehicle worse than coasting to a stop.
//   3. Direction-change interlock: FWD<->REV relay flip is deferred
//      (non-blocking) until the pedal pot is within RELAY_SAFE_MARGIN of
//      neutral. Prevents lurch on gear shift with pedal engaged. Times out
//      after DIR_CHANGE_TIMEOUT_MS if the pedal never releases.
//   4. Decoupled envelope rejection: if only the steer target is out of
//      range, only the steer target snaps to neutral; the accel target is
//      preserved. Same axis-independence for accel. Tai's version reset
//      BOTH on either violation, which caused a bad steer value to release
//      the pedal mid-turn.
//
// ─── STATUS LEDS ────────────────────────────────────────────────────────────
//   LED_ST (D40)  ON when the last received message had an out-of-envelope
//                 value (visible clamp fired). Blinks off on the next clean
//                 message.
//   LED_AC (D41)  ON when the gear is REVERSE (mirrors keyboardmode.ino).
//
// ─── PROVENANCE ─────────────────────────────────────────────────────────────
//   Original manualmode.ino ..... Dinh Ngoc Duc (Duc-san), ISE MRG, 2024
//   motor_control.ino (BCD base). Nguyen Van Tai,          ISE MRG, 2025
//   bcd_automode.ino ............ Fadli Due Ramandavito,   ISE MRG, 2026
// ────────────────────────────────────────────────────────────────────────────

// ─── Tuning ─────────────────────────────────────────────────────────────────

#define SPEED_ST                 255   // Steering motor PWM (0-255)
#define SPEED_AC                 127   // Accel motor PWM (0-255). CAP — higher
                                       // risks damaging the pedal-actuator
                                       // frame per 2026-07-07 manual testing.

#define PM_ST_N                  606   // Steering neutral pot value
#define PM_ST_LIMR               92   // Right envelope: PM_ST_N - LIMR = 200
#define PM_ST_LIML               301   // Left  envelope: PM_ST_N + LIML = 690
#define PM_AC_N                  237   // Accel neutral (pedal fully released)
#define PM_AC_LIMU               524   // Press   envelope: PM_AC_N + LIMU = 735
#define PM_AC_LIMD                20   // Release envelope: PM_AC_N - LIMD = 215

#define REFRESH_MS              1000   // Motor closed-loop keep-alive interval
#define WATCHDOG_MS              500   // Release pedal if no msg for this long
#define STARTUP_HOLD_MS         1000   // Hold neutral for this long at boot
#define RELAY_SAFE_MARGIN         30   // Pedal must be within this of neutral
#define DIR_CHANGE_TIMEOUT_MS   2000   // Max wait for pedal release before
#define MOTOR_TIMEOUT_MS         400
                                       // abandoning a direction change

// ─── Pin Assignments (matches motor_control.ino + keyboardmode.ino) ─────────

const int MD_ST_DIR    =  6;
const int MD_ST_PWM    =  7;
const int MD_AC_DIR    = 11;
const int MD_AC_PWM    = 12;

const int LED_ST       = 40;
const int LED_AC       = 41;
const int RELAY_ALARM  = 50;
const int RELAY_MOTOR  = 51;

const int POT_ST_PIN   = 0;   // A0
const int POT_AC_PIN   = 1;   // A1

// ─── State ──────────────────────────────────────────────────────────────────

int pm_st = 0, pm_ac = 0;                 // Latest raw pot readings

int com_st = PM_ST_N;                     // Current steering pot target
int com_ac = PM_AC_N;                     // Current accel    pot target
int pre_st = 0, pre_ac = 0;               // Previous targets (for closed loop)

unsigned long time_st = 0, time_ac = 0;   // Last motor refresh timestamps
unsigned long last_serial_ms = 0;         // Last VALID message timestamp

String command = "";
String direction_mode = "0";              // Latched gear: "0" FWD, "1" REV
String control_status = "0";              // Echoed back to ROS

bool           envelope_violation   = false;   // For LED_ST
bool           direction_pending    = false;   // Interlock state
String         pending_direction    = "0";
unsigned long  dir_change_start_ms  = 0;
bool           watchdog_was_active  = false;   // Latch: fired since last msg
bool           motor_watchdog_tripped = false;

// ─── Forward Declarations ───────────────────────────────────────────────────

void motor_st(int PM_st_REF);
void motor_ac(int PM_ac_REF);

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
  // 50 ms cap on readStringUntil (early-exits on '\n' — which the Python
  // side always sends). At 10 Hz command rate we have 100 ms/cycle, so
  // 50 ms is a comfortable ceiling for partial-message tolerance.
  Serial.setTimeout(50);
  Serial.begin(115200);

  pinMode(MD_ST_DIR, OUTPUT);   pinMode(MD_ST_PWM, OUTPUT);
  pinMode(MD_AC_DIR, OUTPUT);   pinMode(MD_AC_PWM, OUTPUT);
  pinMode(LED_ST, OUTPUT);      pinMode(LED_AC, OUTPUT);
  pinMode(RELAY_ALARM, OUTPUT); pinMode(RELAY_MOTOR, OUTPUT);

  digitalWrite(MD_ST_DIR, LOW);   analogWrite(MD_ST_PWM, 0);
  digitalWrite(MD_AC_DIR, LOW);   analogWrite(MD_AC_PWM, 0);
  digitalWrite(LED_ST, LOW);      digitalWrite(LED_AC, LOW);
  digitalWrite(RELAY_ALARM, LOW); digitalWrite(RELAY_MOTOR, LOW);   // FWD

  // Startup safety hold: drive both motors to neutral for a moment so we
  // don't accept ROS commands before the pot control loop has converged.
  unsigned long t_boot = millis();
  while (millis() - t_boot < STARTUP_HOLD_MS) {
    motor_st(PM_ST_N);
    motor_ac(PM_AC_N);
  }

  // Prime the watchdog so it doesn't fire before the first message.
  last_serial_ms = millis();
}

// ─── Main Loop ──────────────────────────────────────────────────────────────

void loop() {
  // 1. Parse any pending serial command (non-blocking).
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');     // consumes the '\n'
    command.trim();                             // strips trailing '\r'
    envelope_violation = false;                 // reset for this cycle

    if (command.length() != 10) {
      // Malformed: signal, don't touch com_st/com_ac, don't reset watchdog.
      control_status = "1";
    } else {
      // A clean message arrived. If the watchdog had been firing between
      // the previous message and this one, echo status '2' so the Python
      // side logs a "watchdog tripped" warning — otherwise silent recovery
      // hides the fact that the robot lost heartbeat.
      control_status = (watchdog_was_active || motor_watchdog_tripped) ? "2" : "0";
      watchdog_was_active = false;
      motor_watchdog_tripped = false;
      last_serial_ms = millis();

      String req_direction = command.substring(1, 2);
      int req_com_st = command.substring(2, 5).toInt();
      int req_com_ac = command.substring(5, 8).toInt();

      // Decoupled envelope check. Only the offending axis snaps to neutral —
      // a bad steer value must NOT release the pedal, and vice versa.
      if (req_com_st < PM_ST_N - PM_ST_LIMR || req_com_st > PM_ST_N + PM_ST_LIML) {
        req_com_st = PM_ST_N;
        envelope_violation = true;
      }
      if (req_com_ac < PM_AC_N - PM_AC_LIMD || req_com_ac > PM_AC_N + PM_AC_LIMU) {
        req_com_ac = PM_AC_N;
        envelope_violation = true;
      }

      // Direction change: kick off the non-blocking interlock. We do NOT
      // flip the relay here; that happens in the interlock block below,
      // only after the pedal actually reaches neutral.
      if (req_direction != direction_mode && !direction_pending) {
        direction_pending = true;
        pending_direction = req_direction;
        dir_change_start_ms = millis();
      }

      com_st = req_com_st;
      com_ac = req_com_ac;
    }

    // Reply: status + echo + comma-separated diagnostic tail.
    // hakuroukun_communication_node.py only inspects response[0] for the
    // status char, but the tail is useful when tailing serial by hand.
    String d = (command.length() == 10) ? command.substring(1, 2) : String("0");
    String s = (command.length() == 10) ? command.substring(2, 5) : String("000");
    String a = (command.length() == 10) ? command.substring(5, 8) : String("000");
    Serial.print(control_status);
    Serial.print(d);
    Serial.print(s);
    Serial.print(a);
    Serial.print(",pm_st="); Serial.print(pm_st);
    Serial.print(",pm_ac="); Serial.print(pm_ac);
    Serial.print(",gear="); Serial.print(direction_mode);
    Serial.print(",pend="); Serial.println(direction_pending ? "1" : "0");
    Serial.flush();
  }

  // 2. Direction-change interlock (non-blocking). While pending, we force
  //    the pedal to neutral regardless of what was just parsed, and poll
  //    the accel pot until it's within RELAY_SAFE_MARGIN of neutral.
  if (direction_pending) {
    com_ac = PM_AC_N;
    pm_ac = analogRead(POT_AC_PIN);
    if (abs(pm_ac - PM_AC_N) <= RELAY_SAFE_MARGIN) {
      if (pending_direction == "1") {
        digitalWrite(RELAY_ALARM, HIGH);
        digitalWrite(RELAY_MOTOR, HIGH);
      } else {
        digitalWrite(RELAY_ALARM, LOW);
        digitalWrite(RELAY_MOTOR, LOW);
      }
      direction_mode = pending_direction;
      direction_pending = false;
    } else if (millis() - dir_change_start_ms > DIR_CHANGE_TIMEOUT_MS) {
      // Pedal never released — abandon the direction change, keep old gear
      // latched. Loop continues normally.
      direction_pending = false;
    }
  }

  // 3. Watchdog: no valid message in WATCHDOG_MS -> release pedal.
  //    Steering is intentionally NOT snapped to center — holding the last
  //    valid steer target is safer than centering mid-turn. The latched
  //    flag causes the NEXT successful message to reply with status '2',
  //    so the ROS side can log a "heartbeat lost" warning.
  if (millis() - last_serial_ms > WATCHDOG_MS) {
    com_ac = PM_AC_N;
    watchdog_was_active = true;
  }

  // 4. Drive motors toward their targets.
  motor_st(com_st);
  motor_ac(com_ac);

  // 5. Status LEDs.
  digitalWrite(LED_ST, envelope_violation ? HIGH : LOW);
  digitalWrite(LED_AC, (direction_mode == "1") ? HIGH : LOW);
}

// ─── Steering Motor Closed Loop ─────────────────────────────────────────────
// Drives the steering actuator until the pot reading (A0) matches PM_st_REF.
// The busy-wait is intentional — it comes from Tai's motor_control.ino and
// has proven stable at 10 Hz command rate. Typical convergence < 100 ms.

void motor_st(int PM_st_REF) {
  if (PM_st_REF != pre_st || millis() - time_st > REFRESH_MS) {
    pm_st = analogRead(POT_ST_PIN);
    unsigned long t_start = millis();
    if (pm_st > PM_st_REF) {
      while (pm_st > PM_st_REF) {
        if (millis() - t_start > MOTOR_TIMEOUT_MS) {
          motor_watchdog_tripped = true;
          break;
        }
        digitalWrite(MD_ST_DIR, LOW);
        analogWrite(MD_ST_PWM, SPEED_ST);
        pm_st = analogRead(POT_ST_PIN);
      }
    } else if (pm_st < PM_st_REF) {
      while (pm_st < PM_st_REF) {
        if (millis() - t_start > MOTOR_TIMEOUT_MS) {
          motor_watchdog_tripped = true;
          break;
        }
        digitalWrite(MD_ST_DIR, HIGH);
        analogWrite(MD_ST_PWM, SPEED_ST);
        pm_st = analogRead(POT_ST_PIN);
      }
    }
    analogWrite(MD_ST_PWM, 0);
    digitalWrite(MD_ST_DIR, LOW);
    time_st = millis();
  }
  pre_st = PM_st_REF;
}

// ─── Accel Motor Closed Loop ────────────────────────────────────────────────
// Same structure as motor_st. Note the direction convention: pm_ac INCREASES
// as the pedal is pressed (DIR LOW), and DECREASES as it releases (DIR HIGH).

void motor_ac(int PM_ac_REF) {
  if (PM_ac_REF != pre_ac || millis() - time_ac > REFRESH_MS) {
    pm_ac = analogRead(POT_AC_PIN);
    unsigned long t_start = millis();
    if (pm_ac < PM_ac_REF) {
      while (pm_ac < PM_ac_REF) {
        if (millis() - t_start > MOTOR_TIMEOUT_MS) {
          motor_watchdog_tripped = true;
          break;
        }
        digitalWrite(MD_AC_DIR, LOW);
        analogWrite(MD_AC_PWM, SPEED_AC);
        pm_ac = analogRead(POT_AC_PIN);
      }
    } else if (pm_ac > PM_ac_REF) {
      while (pm_ac > PM_ac_REF) {
        if (millis() - t_start > MOTOR_TIMEOUT_MS) {
          motor_watchdog_tripped = true;
          break;
        }
        digitalWrite(MD_AC_DIR, HIGH);
        analogWrite(MD_AC_PWM, SPEED_AC);
        pm_ac = analogRead(POT_AC_PIN);
      }
    }
    analogWrite(MD_AC_PWM, 0);
    digitalWrite(MD_AC_DIR, LOW);
    time_ac = millis();
  }
  pre_ac = PM_ac_REF;
}
