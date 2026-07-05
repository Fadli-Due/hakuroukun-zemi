// bcd_automode.ino
// Purpose-built Arduino Mega 2560 firmware for BCD autonomous coverage runs.
//
// Designed to pair with:
//   - hakuroukun_communication_node.py  (sends serial commands)
//   - bringup_hakuroukun_robot.launch   (brings up sensors + this node)
//   - offline_path_planning_real.launch (planner + follower + replanner)
//
// Serial protocol (115200 baud):
//   RX from Python:  "0<dir:1><steer:3><accel:3>00\n"   (10 data chars + \n)
//   TX to Python:    "<status:1><dir:1><steer:3><accel:3>,pm_st=<N>,pm_ac=<N>\n"
//
//   dir:    '0' = forward, '1' = reverse
//   steer:  3-digit potentiometer target (valid: 390–885)
//   accel:  3-digit potentiometer target (valid: 270–680)
//   status: '0' = OK, '1' = bad command, '2' = motor watchdog tripped
//
// Differences from motor_control.ino:
//   1. readStringUntil('\n') — proper single-char terminator, no timeout gambling
//   2. Motor watchdog: while-loops break after MOTOR_TIMEOUT_MS to prevent hangs
//   3. SPEED_AC = 180 (70% PWM) — less aggressive than 255, avoids pot overshoot
//   4. Diagnostic response includes live pot readings for field debugging
//   5. Heartbeat LED blink confirms firmware is alive even with no serial traffic
//
// Original motor driver code by Duc-san (c) 2024 ISE Mobile Robot Group
// bcd_automode.ino by Fadli Due Ramandavito, July 2026

// ─── Tuning ─────────────────────────────────────────────────────────────────

#define SPEED_ST          255   // Steering motor PWM (0-255) — fast is fine, steering has hard stops
#define SPEED_AC          180   // Accel motor PWM — 70%, gentler than 255 to avoid pot overshoot
#define PM_ST_N           525   // Steering neutral potentiometer (safety fallback only;
                                //   Python calibrates its own CW=525 / CCW=610 centers)
#define PM_ST_LIMR        200   // Right travel limit offset from neutral
#define PM_ST_LIML        290   // Left travel limit offset from neutral
#define PM_AC_N           290   // Accel neutral potentiometer (no throttle)
#define PM_AC_LIMU        390   // Max throttle offset from neutral
#define PM_AC_LIMD         20   // Min (pedal release) offset from neutral
#define MOTOR_TIMEOUT_MS  400   // Max time (ms) a motor while-loop can run before breaking out
#define MOTOR_REFRESH_MS 1000   // Re-drive motors even if command unchanged (fights pot drift)

// ─── Pin Assignments (identical to motor_control.ino / manualmode.ino) ──────

const int MD_ST_DIR    =  6;
const int MD_ST_PWM    =  7;
const int MD_AC_DIR    = 11;
const int MD_AC_PWM    = 12;

const int LED_ST       = 40;
const int LED_AC       = 41;

const int RELAY_ALARM  = 50;
const int RELAY_MOTOR  = 51;

// ─── State ──────────────────────────────────────────────────────────────────

int com_st = PM_ST_N;          // Current steering target
int com_ac = PM_AC_N;          // Current accel target
int pre_st = 0;                // Previous steering target (for change detection)
int pre_ac = 0;                // Previous accel target
unsigned long time_st = 0;     // Last steering drive timestamp
unsigned long time_ac = 0;     // Last accel drive timestamp

String direction      = "0";  // '0' forward, '1' reverse
String direction_mode = "0";  // Tracks relay state to avoid redundant switching

// Watchdog status — '0' = OK, '2' = a motor timed out on last cycle
char motor_status = '0';

// Heartbeat
unsigned long last_heartbeat = 0;
bool led_state = false;

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);       // 50ms read timeout — generous, \n arrives fast

  pinMode(MD_ST_DIR, OUTPUT);
  pinMode(MD_ST_PWM, OUTPUT);
  pinMode(MD_AC_DIR, OUTPUT);
  pinMode(MD_AC_PWM, OUTPUT);

  pinMode(LED_ST, OUTPUT);
  pinMode(LED_AC, OUTPUT);

  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_MOTOR, OUTPUT);

  // Start in safe state
  digitalWrite(MD_ST_DIR, LOW);
  digitalWrite(MD_ST_PWM, LOW);
  digitalWrite(MD_AC_DIR, LOW);
  digitalWrite(MD_AC_PWM, LOW);

  digitalWrite(LED_ST, LOW);
  digitalWrite(LED_AC, LOW);

  // Relays off = forward mode
  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);

  Serial.println("bcd_automode ready");
}

// ─── Main Loop ──────────────────────────────────────────────────────────────

void loop() {
  char cmd_status = '0';       // Per-cycle command parse status

  // ── 1. Read serial command ──
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    // Strip trailing \r if present (Python may send \r\n)
    if (command.length() > 0 && command.charAt(command.length() - 1) == '\r') {
      command = command.substring(0, command.length() - 1);
    }

    if (command.length() != 10) {
      cmd_status = '1';        // Bad length — keep previous com_st/com_ac
    } else {
      // Parse fields:  0 <dir:1> <steer:3> <accel:3> 00
      //                ^0 ^1     ^2-4      ^5-7      ^8-9
      direction = command.substring(1, 2);

      String s_steer = command.substring(2, 5);
      String s_accel = command.substring(5, 8);

      int new_st = s_steer.toInt();
      int new_ac = s_accel.toInt();

      // Range check — reject obviously wrong values without clobbering state
      bool st_ok = (new_st >= PM_ST_N - PM_ST_LIMR) && (new_st <= PM_ST_N + PM_ST_LIML);
      bool ac_ok = (new_ac >= PM_AC_N - PM_AC_LIMD) && (new_ac <= PM_AC_N + PM_AC_LIMU);

      if (st_ok && ac_ok) {
        com_st = new_st;
        com_ac = new_ac;
      } else {
        // Out of range — hold previous targets, don't reset to neutral.
        // (motor_control.ino reset BOTH to neutral here, which caused
        //  unnecessary steering jumps when only one axis was marginally
        //  out of range.)
        cmd_status = '1';
      }
    }

    // ── 2. Send diagnostic response ──
    int pm_st_now = analogRead(0);
    int pm_ac_now = analogRead(1);
    // Format: <status><dir><steer><accel>,pm_st=<N>,pm_ac=<N>
    Serial.print(cmd_status);
    Serial.print(direction);
    Serial.print(com_st);
    Serial.print(com_ac);
    Serial.print(",pm_st=");
    Serial.print(pm_st_now);
    Serial.print(",pm_ac=");
    Serial.println(pm_ac_now);
    Serial.flush();
  }

  // ── 3. Relay switching (forward / reverse) ──
  if (direction == "1") {
    switch_backward();
  } else {
    switch_forward();
  }

  // ── 4. Drive motors ──
  motor_status = '0';
  motor_st(com_st);
  motor_ac(com_ac);

  // ── 5. Heartbeat LED (pin 13 or LED_ST) — blink every 500ms ──
  if (millis() - last_heartbeat >= 500) {
    last_heartbeat = millis();
    led_state = !led_state;
    digitalWrite(LED_ST, led_state ? HIGH : LOW);
  }
}

// ─── Relay Control ──────────────────────────────────────────────────────────

void switch_backward() {
  if (direction_mode == "1") return;   // Already in reverse
  digitalWrite(RELAY_ALARM, HIGH);
  digitalWrite(RELAY_MOTOR, HIGH);
  direction_mode = "1";
}

void switch_forward() {
  if (direction_mode == "0") return;   // Already in forward
  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  direction_mode = "0";
}

// ─── Motor Drivers with Watchdog ────────────────────────────────────────────
//
// These drive the potentiometer to a target reading using a closed-loop
// while-loop. The MOTOR_TIMEOUT_MS watchdog prevents hangs if:
//   - the pot cable is loose (analogRead floats)
//   - a mechanical stop is reached before the target
//   - pot noise keeps oscillating around the target
//
// When the watchdog fires, motor_status is set to '2' so the Python side
// can log a warning.

void motor_st(int pm_st_ref) {
  if (pm_st_ref != pre_st || millis() - time_st > MOTOR_REFRESH_MS) {
    int pm_st = analogRead(0);
    unsigned long start = millis();

    if (pm_st > pm_st_ref) {
      while (pm_st > pm_st_ref) {
        if (millis() - start > MOTOR_TIMEOUT_MS) {
          motor_status = '2';
          break;
        }
        digitalWrite(MD_ST_DIR, LOW);
        analogWrite(MD_ST_PWM, SPEED_ST);
        pm_st = analogRead(0);
      }
    } else if (pm_st < pm_st_ref) {
      while (pm_st < pm_st_ref) {
        if (millis() - start > MOTOR_TIMEOUT_MS) {
          motor_status = '2';
          break;
        }
        digitalWrite(MD_ST_DIR, HIGH);
        analogWrite(MD_ST_PWM, SPEED_ST);
        pm_st = analogRead(0);
      }
    }

    analogWrite(MD_ST_PWM, 0);
    digitalWrite(MD_ST_DIR, LOW);
    time_st = millis();
  }
  pre_st = pm_st_ref;
}

void motor_ac(int pm_ac_ref) {
  if (pm_ac_ref != pre_ac || millis() - time_ac > MOTOR_REFRESH_MS) {
    int pm_ac = analogRead(1);
    unsigned long start = millis();

    if (pm_ac < pm_ac_ref) {
      while (pm_ac < pm_ac_ref) {
        if (millis() - start > MOTOR_TIMEOUT_MS) {
          motor_status = '2';
          break;
        }
        digitalWrite(MD_AC_DIR, LOW);
        analogWrite(MD_AC_PWM, SPEED_AC);
        pm_ac = analogRead(1);
      }
    } else if (pm_ac > pm_ac_ref) {
      while (pm_ac > pm_ac_ref) {
        if (millis() - start > MOTOR_TIMEOUT_MS) {
          motor_status = '2';
          break;
        }
        digitalWrite(MD_AC_DIR, HIGH);
        analogWrite(MD_AC_PWM, SPEED_AC);
        pm_ac = analogRead(1);
      }
    }

    analogWrite(MD_AC_PWM, 0);
    digitalWrite(MD_AC_DIR, LOW);
    time_ac = millis();
  }
  pre_ac = pm_ac_ref;
}
