// keyboard_mode.ino
// Manual keyboard control for Hakuroukun via serial commands from keyboard_teleop.py
// Flash this to Arduino Mega 2560 (/dev/arduino) instead of manualmode.ino
//
// Commands received over Serial (9600 baud):
//   'w' = accelerate (forward in FORWARD gear, backward in REVERSE gear)
//   'r' = steer right
//   'l' = steer left
//   'b' = depress pedal (release throttle, works in both gears)
//   's' = toggle gear (FORWARD <-> REVERSE) — relay switches ONLY if pedal is at neutral
//   ' ' = stop all  ← keyboard_teleop.py sends this when no key is held
//
// Gear / relay behaviour:
//   FORWARD gear  : RELAY_ALARM=LOW,  RELAY_MOTOR=LOW
//   REVERSE gear  : RELAY_ALARM=HIGH, RELAY_MOTOR=HIGH
//   LED_AC solid ON  → currently in REVERSE gear
//   LED_AC solid OFF → currently in FORWARD gear  (blinks = at pot limit)
//
// SAFETY INTERLOCK: 's' (gear toggle) is silently ignored if pm_ac is above
//   PM_AC_N + RELAY_SAFE_MARGIN, i.e. while the pedal is still pressed.
//   Release pedal first ('b' or stop key), then press 's' to switch gear.
//
// Original manualmode.ino by Duc-san (c) 2024 ISE Mobile Robot Group
// keyboard_mode.ino adapted by Fadli Due Ramandavito, 2026

// --- Tuning (identical to manualmode.ino) --------------------------------
#define SPEED_ST          255   // Steering motor PWM speed (0-255)
#define SPEED_AC          127   // Accel motor PWM speed (0-255)
#define PM_ST_N           400   // Neutral steering potentiometer value
#define PM_ST_LIMR        200   // Right travel limit (PM_ST_N - PM_ST_LIMR)
#define PM_ST_LIML        290   // Left travel limit  (PM_ST_N + PM_ST_LIML)
#define PM_AC_N           290   // Neutral accel potentiometer value
#define PM_AC_LIMU        420   // Pedal-press limit  (PM_AC_N + PM_AC_LIMU)
#define PM_AC_LIMD         20   // Pedal-release limit(PM_AC_N - PM_AC_LIMD)

// Pedal must be within this margin of neutral for relay to switch.
// Set conservatively — if pm_ac < PM_AC_N + RELAY_SAFE_MARGIN the pedal is
// considered safely depressed / at neutral.
#define RELAY_SAFE_MARGIN  30   // ~7% of full pedal travel above neutral

// How often to print PM readings to serial (ms)
#define PM_PRINT_INTERVAL 500

// --- Motor driver pins (identical to manualmode.ino) ---------------------
const int MD_ST_DIR = 6;
const int MD_ST_PWM = 7;
const int MD_AC_DIR = 11;
const int MD_AC_PWM = 12;

// --- LEDs and relays (identical to manualmode.ino) -----------------------
const int LED_ST      = 40;
const int LED_AC      = 41;
const int RELAY_ALARM = 50;
const int RELAY_MOTOR = 51;

// --- Gear state ----------------------------------------------------------
// false = FORWARD (relays LOW), true = REVERSE (relays HIGH)
bool reverse_gear = false;

// --- Per-loop signals ----------------------------------------------------
int pm_st = 0, pm_ac = 0;
int sig_st_r = 0, sig_st_l = 0, sig_ac_u = 0, sig_ac_d = 0;
int light_st = 0;   // Steering LED (blink = at limit)

char cmd = ' ';     // Latest command; default = stop
unsigned long last_pm_print = 0;

// -------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(MD_ST_DIR, OUTPUT);
  pinMode(MD_ST_PWM, OUTPUT);
  pinMode(MD_AC_DIR, OUTPUT);
  pinMode(MD_AC_PWM, OUTPUT);

  pinMode(LED_ST, OUTPUT);
  pinMode(LED_AC, OUTPUT);

  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_MOTOR, OUTPUT);

  // Safe default: FORWARD gear, motors off
  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(LED_AC, LOW);

  Serial.println("keyboard_mode ready.");
  Serial.println("  w=accel  r=steer-right  l=steer-left  b=depress-pedal");
  Serial.println("  s=toggle-gear(interlock)  space=stop");
}

// -------------------------------------------------------------------------
void loop() {
  // 1. Read potentiometers
  pm_st = analogRead(0);
  pm_ac = analogRead(1);

  // 2. Drain serial buffer — keep only the most recent command
  while (Serial.available() > 0) {
    cmd = Serial.read();
  }

  // 3. Reset per-loop signals
  light_st = 0;
  sig_st_r = 0;
  sig_st_l = 0;
  sig_ac_u = 0;
  sig_ac_d = 0;

  // 4. Decode command
  if (cmd == 'r') {
    // ── Steer right ──────────────────────────────────────────────────
    if (pm_st < PM_ST_N - PM_ST_LIMR) {
      light_st = 1;   // At limit
    } else {
      sig_st_r = 1;
    }
  }
  else if (cmd == 'l') {
    // ── Steer left ───────────────────────────────────────────────────
    if (pm_st > PM_ST_N + PM_ST_LIML) {
      light_st = 1;   // At limit
    } else {
      sig_st_l = 1;
    }
  }
  else if (cmd == 'w') {
    // ── Accelerate (direction depends on current gear) ───────────────
    // sig_ac_u presses the pedal regardless of gear.
    // The relay state (already applied below) determines which direction
    // the physical machine moves.
    if (pm_ac > PM_AC_N + PM_AC_LIMU) {
      // Pedal motor at limit — nothing to do (LED_AC handled by gear state)
    } else {
      sig_ac_u = 1;
    }
  }
  else if (cmd == 'b') {
    // ── Depress pedal (works in both gears) ──────────────────────────
    if (pm_ac < PM_AC_N - PM_AC_LIMD) {
      // Already at neutral / released
    } else {
      sig_ac_d = 1;
    }
  }
  else if (cmd == 's') {
    // ── Toggle gear — ONLY if pedal is at/near neutral ───────────────
    bool pedal_safe = (pm_ac <= PM_AC_N + RELAY_SAFE_MARGIN);
    if (pedal_safe) {
      reverse_gear = !reverse_gear;
      if (reverse_gear) {
        digitalWrite(RELAY_ALARM, HIGH);
        digitalWrite(RELAY_MOTOR, HIGH);
        Serial.println(">> REVERSE gear engaged");
      } else {
        digitalWrite(RELAY_ALARM, LOW);
        digitalWrite(RELAY_MOTOR, LOW);
        Serial.println(">> FORWARD gear engaged");
      }
      cmd = ' ';    // Consume the toggle — don't re-fire next loop
    } else {
      // Interlock active — warn operator
      Serial.println("!! Gear switch blocked: release pedal first (press b)");
    }
  }
  // cmd == ' ' or anything else: all signals stay 0 → motors stop

  // 5. Drive steering motor
  if (sig_st_r) {
    digitalWrite(MD_ST_DIR, LOW);
    analogWrite(MD_ST_PWM, SPEED_ST);
  }
  else if (sig_st_l) {
    digitalWrite(MD_ST_DIR, HIGH);
    analogWrite(MD_ST_PWM, SPEED_ST);
  }
  else {
    analogWrite(MD_ST_PWM, 0);
    digitalWrite(MD_ST_DIR, LOW);
  }

  // 6. Drive accel motor
  if (sig_ac_u) {
    digitalWrite(MD_AC_DIR, LOW);
    analogWrite(MD_AC_PWM, SPEED_AC);
  }
  else if (sig_ac_d) {
    digitalWrite(MD_AC_DIR, HIGH);
    analogWrite(MD_AC_PWM, SPEED_AC);
  }
  else {
    analogWrite(MD_AC_PWM, 0);
    digitalWrite(MD_AC_DIR, LOW);
  }

  // 7. LEDs
  //    LED_ST: blink = steering at limit
  //    LED_AC: solid ON = REVERSE gear, OFF = FORWARD gear
  //            (overrides "at limit" blink for accel — gear state is more important)
  digitalWrite(LED_ST, light_st ? HIGH : LOW);
  digitalWrite(LED_AC, reverse_gear ? HIGH : LOW);

  // 8. Print PM readings at most once every PM_PRINT_INTERVAL ms
  unsigned long now = millis();
  if (now - last_pm_print >= PM_PRINT_INTERVAL) {
    Serial.print(reverse_gear ? "REV" : "FWD");
    Serial.print("  pm_st=");
    Serial.print(pm_st);
    Serial.print("  pm_ac=");
    Serial.println(pm_ac);
    last_pm_print = now;
  }

  delay(50);
}
