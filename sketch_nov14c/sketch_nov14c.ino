/*
  Professional — 2-axis Solar Tracker (wider vertical tilt)
  - Target-based movement (adds/subtracts toward target)
  - Top-sensor swap support (fix inverted top-left/top-right)
  - Inversion flags for servo mounting/wiring polarity
  - Faster movement settings
  - Vertical range expanded for larger tilt (SERVO2_MIN/ MAX)
  
  WARNING: Ensure your mechanical bracket allows the expanded tilt range.
           Avoid hitting physical stops to prevent servo stress.
*/

#include <Wire.h>
#include <Servo.h>
#include <lcdgfx.h>  // OLED SSD1306 - SH1106 library

// --- OLED -----------------------------------------------------------------
DisplaySH1106_128x64_I2C display(-1); // SH1106 128x64, no reset pin

// --- Hardware pins --------------------------------------------------------
Servo servoH; // Horizontal (azimuth)
Servo servoV; // Vertical (elevation)

// LDR pins (analog)
const int LDR_TOP_LEFT     = A0;
const int LDR_TOP_RIGHT    = A1;
const int LDR_BOTTOM_LEFT  = A2;
const int LDR_BOTTOM_RIGHT = A3;

// Servo PWM pins (digital)
const int SERVO_HORIZONTAL_PIN = 9;
const int SERVO_VERTICAL_PIN   = 8;

// --- Servo angle limits (safe 0..180 sub-range) ---------------------------
const int SERVO1_MIN = 20;    // Horizontal min angle
const int SERVO1_MAX = 170;   // Horizontal max angle

// USER REQUEST: Increased max tilt (vertical)
// Moderately wide tilt: adjust if your mechanics allow more/less
const int SERVO2_MIN = 5;     // Vertical min angle (increased range)
const int SERVO2_MAX = 175;   // Vertical max angle (increased range)

// --- Initial positions (center of allowed ranges) -------------------------
int posH = (SERVO1_MIN + SERVO1_MAX) / 2;
int posV = (SERVO2_MIN + SERVO2_MAX) / 2;

// --- Behaviour & smoothing tuning -----------------------------------------
const int SAMPLES = 6;                // analog readings to average (faster response)
const int DELAY_BETWEEN_SAMPLES = 3;  // ms between analog samples
const int LOOP_DELAY = 60;            // ms between main loops

const int MAX_STEP = 8;               // max degrees to move per loop toward target (faster)
const float H_GAIN = 1.0f;            // horizontal sensitivity multiplier
const float V_GAIN = 1.0f;            // vertical sensitivity multiplier

const int DEADZONE = 8;               // small deadzone in sensor ratio (reduces twitch)

// Inversion flags: set true if the physical servo responds opposite direction
const bool INVERT_HORIZONTAL = true;
const bool INVERT_VERTICAL   = true;

// If the top-left and top-right physical sensors are swapped in wiring/layout,
// enable to swap their readings before computing left/right sums:
const bool SWAP_TOP_SENSORS = true;

// --- OLED layout constants -----------------------------------------------
const int LABEL_Y_TOP     = 0;   // Y for top labels
const int VALUE_Y_TOP     = 10;  // Y for top values
const int LABEL_Y_BOTTOM  = 24;  // Y for bottom labels
const int VALUE_Y_BOTTOM  = 34;  // Y for bottom values
const int RIGHT_COLUMN_X  = 80;  // X start for right column values

// Helper buffer for formatted values
char valbuf[16];

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  servoH.attach(SERVO_HORIZONTAL_PIN);
  servoV.attach(SERVO_VERTICAL_PIN);

  // Constrain and move to initial safe positions
  posH = constrain(posH, SERVO1_MIN, SERVO1_MAX);
  posV = constrain(posV, SERVO2_MIN, SERVO2_MAX);
  servoH.write(posH);
  servoV.write(posV);
  delay(600);

  Serial.println("Professional 2-axis Solar Tracker (wider vertical tilt)");
  Serial.print("H range: "); Serial.print(SERVO1_MIN); Serial.print(" - "); Serial.println(SERVO1_MAX);
  Serial.print("V range: "); Serial.print(SERVO2_MIN); Serial.print(" - "); Serial.println(SERVO2_MAX);

  // ---------------- OLED init (ADDED) ----------------
  display.begin();
  display.clear();
  display.setFixedFont(ssd1306xled_font6x8);

  // Draw initial static labels (these will be redrawn in loop)
  display.printFixed(0, LABEL_Y_TOP,    " TL     |  LDR TR", STYLE_NORMAL);
  display.printFixed(0, LABEL_Y_BOTTOM, " LDR BL     |  LDR BTMR", STYLE_NORMAL);

  // Initial placeholders
  display.printFixed(12, VALUE_Y_TOP,   "----", STYLE_NORMAL);
  display.printFixed(RIGHT_COLUMN_X + 8, VALUE_Y_TOP, "----", STYLE_NORMAL);
  display.printFixed(12, VALUE_Y_BOTTOM, "----", STYLE_NORMAL);
  display.printFixed(RIGHT_COLUMN_X + 8, VALUE_Y_BOTTOM, "----", STYLE_NORMAL);
  // --------------------------------------------------
}

// Read-and-average helper
int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    s += analogRead(pin);
    delay(DELAY_BETWEEN_SAMPLES);
  }
  return (int)(s / SAMPLES);
}

// Map a -1..+1 ratio to an angle offset within halfRange (float)
float ratioToAngleOffset(float ratio, float gain, float halfRange) {
  if (ratio > 1.0f) ratio = 1.0f;
  if (ratio < -1.0f) ratio = -1.0f;
  return ratio * gain * halfRange;
}

void loop() {
  // --- read sensors (averaged) --------------------------------------------
  int tl = readAvg(LDR_TOP_LEFT);
  int tr = readAvg(LDR_TOP_RIGHT);
  int bl = readAvg(LDR_BOTTOM_LEFT);
  int br = readAvg(LDR_BOTTOM_RIGHT);

  // If top sensors are physically inverted, swap them to correct logical layout
  if (SWAP_TOP_SENSORS) {
    int tmp = tl;
    tl = tr;
    tr = tmp;
  }

  // ---------------- OLED: redraw labels + raw values (ADDED) -------------
  // Clear entire display (safe & compatible with lcdgfx)
  display.clear();

  // Static labels
  display.printFixed(0, LABEL_Y_TOP,    " TL | TR", STYLE_NORMAL);
  display.printFixed(0, LABEL_Y_BOTTOM, " BL | BR", STYLE_NORMAL);

  // Format and print values aligned under labels (4-digit aligned)
  snprintf(valbuf, sizeof(valbuf), "%4d", tl);
  display.printFixed(12, VALUE_Y_TOP, valbuf, STYLE_NORMAL);

  snprintf(valbuf, sizeof(valbuf), "%4d", tr);
  display.printFixed(RIGHT_COLUMN_X + 8, VALUE_Y_TOP, valbuf, STYLE_NORMAL);

  snprintf(valbuf, sizeof(valbuf), "%4d", bl);
  display.printFixed(12, VALUE_Y_BOTTOM, valbuf, STYLE_NORMAL);

  snprintf(valbuf, sizeof(valbuf), "%4d", br);
  display.printFixed(RIGHT_COLUMN_X + 8, VALUE_Y_BOTTOM, valbuf, STYLE_NORMAL);
  // ----------------------------------------------------------------------

  // Compute sums used for normalized ratios
  int leftSum  = tl + bl;
  int rightSum = tr + br;
  int topSum   = tl + tr;
  int bottomSum= bl + br;

  // Compute normalized ratios in range -1..1
  float ratioH = 0.0f; // +1 => more left, -1 => more right
  float ratioV = 0.0f; // +1 => more top,  -1 => more bottom

  if ((leftSum + rightSum) > 0) {
    ratioH = float(leftSum - rightSum) / float(leftSum + rightSum);
  }
  if ((topSum + bottomSum) > 0) {
    ratioV = float(topSum - bottomSum) / float(topSum + bottomSum);
  }

  // Debug raw values
  Serial.print("TL:"); Serial.print(tl);
  Serial.print(" TR:"); Serial.print(tr);
  Serial.print(" BL:"); Serial.print(bl);
  Serial.print(" BR:"); Serial.print(br);

  // Apply small deadzone to avoid tiny random adjustments
  if (fabs(ratioH) * 1023.0f < DEADZONE) ratioH = 0.0f;
  if (fabs(ratioV) * 1023.0f < DEADZONE) ratioV = 0.0f;

  // --- compute targets ----------------------------------------------------
  const float centerH = float(SERVO1_MIN + SERVO1_MAX) / 2.0f;
  const float centerV = float(SERVO2_MIN + SERVO2_MAX) / 2.0f;
  const float halfRangeH = float(SERVO1_MAX - SERVO1_MIN) / 2.0f;
  const float halfRangeV = float(SERVO2_MAX - SERVO2_MIN) / 2.0f;

  // inversion multipliers (if the servo is physically reversed)
  const int invH = INVERT_HORIZONTAL ? -1 : 1;
  const int invV = INVERT_VERTICAL   ? -1 : 1;

  // Horizontal mapping:
  // ratioH positive => more light on left => we want left movement (decrease angle from center).
  float offsetH = ratioToAngleOffset(ratioH, H_GAIN, halfRangeH);
  float targetH = centerH - (float)invH * offsetH;

  // Vertical mapping:
  // ratioV positive => more light on top => we want tilt up (increase angle from center).
  float offsetV = ratioToAngleOffset(ratioV, V_GAIN, halfRangeV);
  float targetV = centerV + (float)invV * offsetV;

  // Constrain targets to safe servo ranges
  targetH = constrain(targetH, float(SERVO1_MIN), float(SERVO1_MAX));
  targetV = constrain(targetV, float(SERVO2_MIN), float(SERVO2_MAX));

  // --- move servos toward targets by limited step (add/subtract) ----------
  // Horizontal
  float diffH = targetH - float(posH);
  if (fabs(diffH) >= 0.5f) {
    int step = (int)min(float(MAX_STEP), ceil(fabs(diffH)));
    if (diffH > 0) posH += step;
    else           posH -= step;
    posH = constrain(posH, SERVO1_MIN, SERVO1_MAX);
    servoH.write(posH);
  }

  // Vertical
  float diffV = targetV - float(posV);
  if (fabs(diffV) >= 0.5f) {
    int step = (int)min(float(MAX_STEP), ceil(fabs(diffV)));
    if (diffV > 0) posV += step;
    else           posV -= step;
    posV = constrain(posV, SERVO2_MIN, SERVO2_MAX);
    servoV.write(posV);
  }

  // --- debug printing -----------------------------------------------------
  Serial.print(" | ratioH:"); Serial.print(ratioH, 3);
  Serial.print(" ratioV:"); Serial.print(ratioV, 3);
  Serial.print(" | targetH:"); Serial.print(targetH, 1);
  Serial.print(" targetV:"); Serial.print(targetV, 1);
  Serial.print(" | posH:"); Serial.print(posH);
  Serial.print(" posV:"); Serial.println(posV);

  delay(LOOP_DELAY);
}
