// keyboardmode.ino
// Manual keyboard control for Hakuroukun via serial commands from keyboard_teleop.py
// Flash to Arduino Mega 2560 (/dev/arduino).
//
// Serial protocol (9600 baud):
//   'w' = press pedal (accelerate)
//   'b' = release pedal
//   'a' = steer left
//   'd' = steer right
//   's' = toggle gear FORWARD <-> REVERSE (interlock: pedal must be at neutral)
//   ' ' = stop all (keyboard_teleop.py sends this when no key is held)
//
// Relay behaviour:
//   FORWARD gear : RELAY_ALARM=LOW,  RELAY_MOTOR=LOW,  LED_AC OFF
//   REVERSE gear : RELAY_ALARM=HIGH, RELAY_MOTOR=HIGH, LED_AC ON
//
// Every loop, prints a single status line to serial:
//   [cmd] KEY_NAME | gear=FWD/REV | pm_st=N | pm_ac=N | signals
// Prints are throttled to PRINT_INTERVAL_MS so the log stays readable.
//
// Original manualmode.ino by Duc-san (c) 2024 ISE Mobile Robot Group
// keyboardmode.ino adapted by Fadli Due Ramandavito, 2026

// ─── Tuning ─────────────────────────────────────────────────────────────────

#define SPEED_ST          255   // Steering motor PWM (0-255)
#define SPEED_AC          127   // Accel motor PWM (0-255)
#define PM_ST_N           400   // Steering neutral pot value
#define PM_ST_LIMR        200   // Right limit: PM_ST_N - PM_ST_LIMR
#define PM_ST_LIML        290   // Left limit:  PM_ST_N + PM_ST_LIML
#define PM_AC_N           235   // measured 2026-07-07: released pot value
#define PM_AC_LIMU        500   // press limit: 235+500=735
#define PM_AC_LIMD         20   // release limit: 235-20=215
#define RELAY_SAFE_MARGIN  30   // Pedal within this of neutral to allow gear switch
#define PRINT_INTERVAL_MS 100   // Throttle status prints (10 Hz)

// ─── Pin Assignments ────────────────────────────────────────────────────────

const int MD_ST_DIR    =  6;
const int MD_ST_PWM    =  7;
const int MD_AC_DIR    = 11;
const int MD_AC_PWM    = 12;
const int LED_ST       = 40;
const int LED_AC       = 41;
const int RELAY_ALARM  = 50;
const int RELAY_MOTOR  = 51;

// ─── State ──────────────────────────────────────────────────────────────────

bool reverse_gear = false;

int pm_st = 0, pm_ac = 0;
int sig_st_r = 0, sig_st_l = 0, sig_ac_u = 0, sig_ac_d = 0;
int light_st = 0;

char cmd = ' ';
unsigned long last_print = 0;

// ─── Setup ──────────────────────────────────────────────────────────────────

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

  // Safe defaults
  digitalWrite(MD_ST_DIR, LOW);  analogWrite(MD_ST_PWM, 0);
  digitalWrite(MD_AC_DIR, LOW);  analogWrite(MD_AC_PWM, 0);
  digitalWrite(LED_ST, LOW);     digitalWrite(LED_AC, LOW);
  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);

  Serial.println("=== keyboardmode ready ===");
  Serial.println("  w = press pedal (accelerate)");
  Serial.println("  b = release pedal");
  Serial.println("  a = steer left     d = steer right");
  Serial.println("  s = toggle gear (requires pedal at neutral)");
  Serial.println("  space = stop all");
}

// ─── Main Loop ──────────────────────────────────────────────────────────────

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
  sig_st_r = sig_st_l = sig_ac_u = sig_ac_d = 0;

  // 4. Decode command
  switch (cmd) {
    case 'd':
      if (pm_st < PM_ST_N - PM_ST_LIMR) light_st = 1;
      else                              sig_st_r = 1;
      break;

    case 'a':
      if (pm_st > PM_ST_N + PM_ST_LIML) light_st = 1;
      else                              sig_st_l = 1;
      break;

    case 'w':
      if (pm_ac <= PM_AC_N + PM_AC_LIMU) sig_ac_u = 1;
      // else: at pedal limit — do nothing
      break;

    case 'b':
      if (pm_ac >= PM_AC_N - PM_AC_LIMD) sig_ac_d = 1;
      // else: already at neutral / released
      break;

    case 's':
      handle_gear_toggle();
      cmd = ' ';   // consume so it doesn't re-fire next loop
      break;

    default:
      // ' ' or anything else -> all signals stay 0 -> motors stop
      break;
  }

  // 5. Drive steering motor
  if (sig_st_r) {
    digitalWrite(MD_ST_DIR, LOW);
    analogWrite(MD_ST_PWM, SPEED_ST);
  } else if (sig_st_l) {
    digitalWrite(MD_ST_DIR, HIGH);
    analogWrite(MD_ST_PWM, SPEED_ST);
  } else {
    analogWrite(MD_ST_PWM, 0);
    digitalWrite(MD_ST_DIR, LOW);
  }

  // 6. Drive accel motor (continuous while held — same as manualmode.ino)
  if (sig_ac_u) {
    digitalWrite(MD_AC_DIR, LOW);
    analogWrite(MD_AC_PWM, SPEED_AC);
  } else if (sig_ac_d) {
    digitalWrite(MD_AC_DIR, HIGH);
    analogWrite(MD_AC_PWM, SPEED_AC);
  } else {
    analogWrite(MD_AC_PWM, 0);
    digitalWrite(MD_AC_DIR, LOW);
  }

  // 7. LEDs
  digitalWrite(LED_ST, light_st       ? HIGH : LOW);
  digitalWrite(LED_AC, reverse_gear   ? HIGH : LOW);

  // 8. Throttled status print
  print_status();

  delay(50);
}

// ─── Gear Toggle with Interlock ─────────────────────────────────────────────

void handle_gear_toggle() {
  bool pedal_safe = (pm_ac <= PM_AC_N + RELAY_SAFE_MARGIN);
  if (!pedal_safe) {
    Serial.println("!! Gear switch BLOCKED: release pedal first (press b)");
    return;
  }
  reverse_gear = !reverse_gear;
  if (reverse_gear) {
    digitalWrite(RELAY_ALARM, HIGH);
    digitalWrite(RELAY_MOTOR, HIGH);
    Serial.println(">> Gear -> REVERSE");
  } else {
    digitalWrite(RELAY_ALARM, LOW);
    digitalWrite(RELAY_MOTOR, LOW);
    Serial.println(">> Gear -> FORWARD");
  }
}

// ─── Status Print ───────────────────────────────────────────────────────────

void print_status() {
  static char last_key = '?';
  static bool last_gear = false;
  static int last_pm_st = -1;
  static int last_pm_ac = -1;

  // Determine current key name
  char current_key;
  if      (sig_ac_u) current_key = 'W';
  else if (sig_ac_d) current_key = 'B';
  else if (sig_st_r) current_key = 'D';
  else if (sig_st_l) current_key = 'A';
  else               current_key = '-';

  // Only print if something meaningful changed OR every 1000ms as heartbeat
  bool state_changed = (current_key != last_key) ||
                       (reverse_gear != last_gear) ||
                       (abs(pm_st - last_pm_st) > 5) ||
                       (abs(pm_ac - last_pm_ac) > 5);

  if (!state_changed && millis() - last_print < 1000) return;

  last_print = millis();
  last_key = current_key;
  last_gear = reverse_gear;
  last_pm_st = pm_st;
  last_pm_ac = pm_ac;

  const char *key_name;
  switch (current_key) {
    case 'W': key_name = "[W] ACCEL  "; break;
    case 'B': key_name = "[B] RELEASE"; break;
    case 'D': key_name = "[D] RIGHT  "; break;
    case 'A': key_name = "[A] LEFT   "; break;
    default:  key_name = "[-] STOP   "; break;
  }

  Serial.print(key_name);
  Serial.print(" | gear=");
  Serial.print(reverse_gear ? "REV" : "FWD");
  Serial.print(" | pm_st=");
  Serial.print(pm_st);
  Serial.print(" | pm_ac=");
  Serial.print(pm_ac);
  if (light_st) Serial.print(" | STEER-LIMIT");
  Serial.println();
}
