
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>

/*
  ROBOT CANDIDATES 2026
  ESP32-WROOM-32 + L298N + 4 TT motors + 3 HC-SR04P
  + TCRT5000 3-channel + TCS34725 + LCD 20x4 + servo.

  IMPORTANT:
  - This is an ADVANCE VERSION intended to be calibrated on the real robot.
  - It deliberately does NOT include the optional ArUco or return-path bonuses.
  - ACTIVE_TRACK selects which competition track the robot runs.
  - No track layout is hard-coded. The maze/navigation logic builds information
    dynamically from the ultrasonic sensors.
  - The robot has no wheel encoders in the current schematic, so distances and
    90-degree turns are time-calibrated. These values MUST be calibrated.
*/

// ============================================================
// 1. SELECT TRACK / MODE
// ============================================================

#define TRACK_A 1
#define TRACK_B 2

// Change ONLY this before the round/calibration period.
#define ACTIVE_TRACK TRACK_A

// Set to 1 while developing to print color and line sensor data.
// Set to 0 for autonomous competition operation.
#define CALIBRATION_MODE 0


// ============================================================
// 2. MOTOR PINS - CURRENT SCHEMATIC
// ============================================================

#define PIN_ENA 14
#define PIN_IN1 27
#define PIN_IN2 25

#define PIN_ENB 32
#define PIN_IN3 26
#define PIN_IN4 33

// If one side is physically reversed, change one of these to true.
// This is NOT a substitute for wiring the two motors on each side correctly.
#define INVERT_LEFT  false
#define INVERT_RIGHT false


// ============================================================
// 3. ULTRASONIC PINS
// ============================================================

#define TRIG_LEFT   18
#define ECHO_LEFT   34

#define TRIG_FRONT  19
#define ECHO_FRONT  36

#define TRIG_RIGHT  21
#define ECHO_RIGHT  35

// HC-SR04P should be powered according to its exact module specification.
// For the user's stated wide-voltage HC-SR04P, 3.3 V operation is intended.
// Do NOT feed a 5 V ECHO into an ESP32 GPIO.


/*
  Approximate wall threshold.

  A unit is 30 cm wide. When the robot is centered, a wall is normally
  around half a unit away. This value MUST be calibrated on the real robot.
*/
const float WALL_DISTANCE_CM = 22.0;


// ============================================================
// 4. I2C
// ============================================================

#define I2C_SDA 22
#define I2C_SCL 23

// Common LCD backpacks use 0x27 or 0x3F.
// Change after running the I2C scanner/calibration.
#define LCD_ADDRESS 0x27

LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

Adafruit_TCS34725 tcs(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);


// ============================================================
// 5. SERVO / GRIPPER
// ============================================================

#define SERVO_PIN 5

Servo gripper;

const int SERVO_OPEN_ANGLE  = 55;
const int SERVO_CLOSE_ANGLE = 125;


// ============================================================
// 6. 3-CHANNEL LINE FOLLOWER / LINE AVOIDER
// ============================================================

#define LINE_LEFT   4
#define LINE_CENTER 16
#define LINE_RIGHT  17

/*
  Many TCRT5000 modules output LOW over the detected line and HIGH otherwise.
  If your module behaves opposite, change LOW to HIGH.
*/
#define LINE_WHITE_LEVEL LOW

int lastLineCorrection = 0; // -1 = left, +1 = right


// ============================================================
// 7. IR SENSORS FROM CURRENT SCHEMATIC
// ============================================================

#define IR_1 15
#define IR_2 2


// ============================================================
// 8. MOVEMENT CALIBRATION
// ============================================================

/*
  These values are starting points only.

  TT motors are specified as 115 RPM at 6 V without load.
  Real speed changes with battery voltage, L298N losses, load, floor,
  wheel diameter and PWM. Therefore these values MUST be measured.
*/

const int BASE_SPEED = 145;
const int TURN_SPEED = 145;

const unsigned long CELL_FORWARD_MS = 1450;
const unsigned long TURN_90_MS       = 430;

const unsigned long SHORT_FORWARD_MS = 250;


// ============================================================
// 9. COLORS
// ============================================================

enum ColorName {
  COLOR_UNKNOWN,
  COLOR_CYAN,
  COLOR_YELLOW,
  COLOR_ORANGE,
  COLOR_PINK,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_WHITE
};

struct RGBSample {
  float r;
  float g;
  float b;
  float h;
  float s;
  float v;
};


// ============================================================
// 10. GRID / HEADING
// ============================================================

enum Heading {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

const int MAP_SIZE = 9;
const int MAP_CENTER = 4;

struct Cell {
  bool visited;
  bool knownWall[4];
  bool wall[4];
};

Cell mapGrid[MAP_SIZE][MAP_SIZE];

int robotX = MAP_CENTER;
int robotY = MAP_CENTER;
Heading heading = NORTH;


// ============================================================
// 11. BASIC MOTOR FUNCTIONS
// ============================================================

void setLeftDirection(bool forward) {
  if (INVERT_LEFT) forward = !forward;

  if (forward) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
}

void setRightDirection(bool forward) {
  if (INVERT_RIGHT) forward = !forward;

  if (forward) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  } else {
    setLeftDirection(leftSpeed > 0);
    analogWrite(PIN_ENA, abs(leftSpeed));
  }

  if (rightSpeed == 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
  } else {
    setRightDirection(rightSpeed > 0);
    analogWrite(PIN_ENB, abs(rightSpeed));
  }
}

void stopMotors() {
  setMotorSpeed(0, 0);
}

void forward(int speed = BASE_SPEED) {
  setMotorSpeed(speed, speed);
}

void backward(int speed = BASE_SPEED) {
  setMotorSpeed(-speed, -speed);
}

void turnLeftInPlace(int speed = TURN_SPEED) {
  setMotorSpeed(-speed, speed);
}

void turnRightInPlace(int speed = TURN_SPEED) {
  setMotorSpeed(speed, -speed);
}


// ============================================================
// 12. MOVEMENT WITH TIME CALIBRATION
// ============================================================

void moveOneCell() {
  forward(BASE_SPEED);
  delay(CELL_FORWARD_MS);
  stopMotors();
  delay(80);
}

void moveShort() {
  forward(BASE_SPEED);
  delay(SHORT_FORWARD_MS);
  stopMotors();
}

void turnLeft90() {
  turnLeftInPlace(TURN_SPEED);
  delay(TURN_90_MS);
  stopMotors();
  delay(80);
  heading = (Heading)((heading + 3) % 4);
}

void turnRight90() {
  turnRightInPlace(TURN_SPEED);
  delay(TURN_90_MS);
  stopMotors();
  delay(80);
  heading = (Heading)((heading + 1) % 4);
}

void turnAround() {
  turnRight90();
  turnRight90();
}

void faceHeading(Heading target) {
  int diff = ((int)target - (int)heading + 4) % 4;

  if (diff == 0) return;
  if (diff == 1) turnRight90();
  else if (diff == 2) turnAround();
  else if (diff == 3) turnLeft90();
}


// ============================================================
// 13. ULTRASONICS
// ============================================================

float readDistanceCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

  if (duration == 0) return 400.0;

  return (duration * 0.0343f) / 2.0f;
}

float distanceLeft() {
  float d = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  delay(25);
  return d;
}

float distanceFront() {
  float d = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
  delay(25);
  return d;
}

float distanceRight() {
  float d = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
  delay(25);
  return d;
}

bool wallAtRelativeLeft() {
  return distanceLeft() < WALL_DISTANCE_CM;
}

bool wallAtRelativeFront() {
  return distanceFront() < WALL_DISTANCE_CM;
}

bool wallAtRelativeRight() {
  return distanceRight() < WALL_DISTANCE_CM;
}


// ============================================================
// 14. GRID HELPERS
// ============================================================

bool insideMap(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

int dxForHeading(Heading h) {
  if (h == EAST) return 1;
  if (h == WEST) return -1;
  return 0;
}

int dyForHeading(Heading h) {
  if (h == NORTH) return 1;
  if (h == SOUTH) return -1;
  return 0;
}

Heading opposite(Heading h) {
  return (Heading)(((int)h + 2) % 4);
}

Heading leftOf(Heading h) {
  return (Heading)(((int)h + 3) % 4);
}

Heading rightOf(Heading h) {
  return (Heading)(((int)h + 1) % 4);
}

void resetMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].visited = false;

      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].knownWall[d] = false;
        mapGrid[x][y].wall[d] = false;
      }
    }
  }

  robotX = MAP_CENTER;
  robotY = MAP_CENTER;
  heading = NORTH;
}

void markWall(int x, int y, Heading d, bool wall) {
  if (!insideMap(x, y)) return;

  mapGrid[x][y].knownWall[d] = true;
  mapGrid[x][y].wall[d] = wall;

  int nx = x + dxForHeading(d);
  int ny = y + dyForHeading(d);

  if (insideMap(nx, ny)) {
    Heading od = opposite(d);
    mapGrid[nx][ny].knownWall[od] = true;
    mapGrid[nx][ny].wall[od] = wall;
  }
}

void measureWallsAtCurrentCell() {
  /*
    We measure front, left, right, then turn around to measure the rear,
    then restore the original heading.
  */

  bool frontWall = wallAtRelativeFront();
  bool leftWall  = wallAtRelativeLeft();
  bool rightWall = wallAtRelativeRight();

  Heading original = heading;

  turnAround();
  bool rearWall = wallAtRelativeFront();
  faceHeading(original);

  markWall(robotX, robotY, original, frontWall);
  markWall(robotX, robotY, leftOf(original), leftWall);
  markWall(robotX, robotY, rightOf(original), rightWall);
  markWall(robotX, robotY, opposite(original), rearWall);

  mapGrid[robotX][robotY].visited = true;
}

int countKnownWalls(int x, int y) {
  int count = 0;

  for (int d = 0; d < 4; d++) {
    if (mapGrid[x][y].knownWall[d] && mapGrid[x][y].wall[d]) {
      count++;
    }
  }

  return count;
}

bool accessibleAndUnvisited(Heading d) {
  if (!mapGrid[robotX][robotY].knownWall[d]) return false;
  if (mapGrid[robotX][robotY].wall[d]) return false;

  int nx = robotX + dxForHeading(d);
  int ny = robotY + dyForHeading(d);

  if (!insideMap(nx, ny)) return false;

  return !mapGrid[nx][ny].visited;
}

void updateLogicalPosition(Heading movementDirection) {
  robotX += dxForHeading(movementDirection);
  robotY += dyForHeading(movementDirection);

  if (!insideMap(robotX, robotY)) {
    robotX = constrain(robotX, 0, MAP_SIZE - 1);
    robotY = constrain(robotY, 0, MAP_SIZE - 1);
  }
}

void moveToNeighbor(Heading target) {
  faceHeading(target);
  moveOneCell();
  updateLogicalPosition(target);
}


// ============================================================
// 15. COLOR SENSOR
// ============================================================

RGBSample readColorSample() {
  float r, g, b;
  tcs.getRGB(&r, &g, &b);

  RGBSample out;
  out.r = r;
  out.g = g;
  out.b = b;

  float maxv = max(r, max(g, b));
  float minv = min(r, min(g, b));
  float delta = maxv - minv;

  out.v = maxv;

  if (maxv <= 0.1f) {
    out.s = 0;
    out.h = 0;
    return out;
  }

  out.s = delta / maxv;

  if (delta < 0.01f) {
    out.h = 0;
  } else if (maxv == r) {
    out.h = 60.0f * fmod(((g - b) / delta), 6.0f);
  } else if (maxv == g) {
    out.h = 60.0f * (((b - r) / delta) + 2.0f);
  } else {
    out.h = 60.0f * (((r - g) / delta) + 4.0f);
  }

  if (out.h < 0) out.h += 360.0f;

  return out;
}

ColorName classifyColor(const RGBSample &c) {
  // These are STARTING thresholds, not final calibrated thresholds.
  if (c.v < 25 || c.s < 0.18f) {
    return COLOR_UNKNOWN;
  }

  float h = c.h;

  // Green start / section colors
  if (h >= 75 && h < 165) return COLOR_GREEN;

  // Cyan
  if (h >= 165 && h < 215) return COLOR_CYAN;

  // Yellow
  if (h >= 45 && h < 75) return COLOR_YELLOW;

  // Orange
  if (h >= 15 && h < 45) return COLOR_ORANGE;

  // Pink / magenta
  if (h >= 295 && h < 350) return COLOR_PINK;

  // Red
  if (h < 15 || h >= 350) return COLOR_RED;

  // White is intentionally conservative.
  if (c.s < 0.12f && c.v > 150) return COLOR_WHITE;

  return COLOR_UNKNOWN;
}

const char* colorName(ColorName c) {
  switch (c) {
    case COLOR_CYAN:   return "CIAN";
    case COLOR_YELLOW: return "AMARILLO";
    case COLOR_ORANGE: return "NARANJA";
    case COLOR_PINK:   return "ROSA";
    case COLOR_RED:    return "ROJO";
    case COLOR_GREEN:  return "VERDE";
    case COLOR_WHITE:  return "BLANCO";
    default:           return "DESCONOCIDO";
  }
}

void showColorOnLCD(ColorName c) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("COLOR DETECTADO");

  lcd.setCursor(0, 1);
  lcd.print(colorName(c));

  lcd.setCursor(0, 2);
  lcd.print("Track: ");
  lcd.print(ACTIVE_TRACK == TRACK_A ? "A MAZE" : "B NIVELES");
}

ColorName readColor() {
  RGBSample c = readColorSample();

  if (CALIBRATION_MODE) {
    Serial.print("RGB=");
    Serial.print(c.r, 1);
    Serial.print(",");
    Serial.print(c.g, 1);
    Serial.print(",");
    Serial.print(c.b, 1);
    Serial.print(" H=");
    Serial.print(c.h, 1);
    Serial.print(" S=");
    Serial.print(c.s, 3);
    Serial.print(" V=");
    Serial.println(c.v, 1);
  }

  return classifyColor(c);
}


// ============================================================
// 16. LINE SENSOR
// ============================================================

bool lineLeft() {
  return digitalRead(LINE_LEFT) == LINE_WHITE_LEVEL;
}

bool lineCenter() {
  return digitalRead(LINE_CENTER) == LINE_WHITE_LEVEL;
}

bool lineRight() {
  return digitalRead(LINE_RIGHT) == LINE_WHITE_LEVEL;
}

void lineAvoidanceStep() {
  bool L = lineLeft();
  bool C = lineCenter();
  bool R = lineRight();

  if (CALIBRATION_MODE) {
    Serial.print("LINE L=");
    Serial.print(L);
    Serial.print(" C=");
    Serial.print(C);
    Serial.print(" R=");
    Serial.println(R);
  }

  // No white detected: go straight.
  if (!L && !C && !R) {
    forward(BASE_SPEED);
    return;
  }

  // White on left: move right.
  if (L && !R) {
    lastLineCorrection = 1;
    setMotorSpeed(BASE_SPEED + 35, BASE_SPEED - 45);
    return;
  }

  // White on right: move left.
  if (R && !L) {
    lastLineCorrection = -1;
    setMotorSpeed(BASE_SPEED - 45, BASE_SPEED + 35);
    return;
  }

  // Center or all sensors see white.
  // Continue the last correction until the white line disappears.
  if (lastLineCorrection > 0) {
    setMotorSpeed(BASE_SPEED + 40, BASE_SPEED - 55);
  } else {
    setMotorSpeed(BASE_SPEED - 55, BASE_SPEED + 40);
  }
}


// ============================================================
// 17. GRIPPER
// ============================================================

void gripperOpen() {
  gripper.write(SERVO_OPEN_ANGLE);
  delay(400);
}

void gripperClose() {
  gripper.write(SERVO_CLOSE_ANGLE);
  delay(500);
}

bool approachBallAndGrab() {
  /*
    Advance version:
    the front ultrasonic is used to find an object in front.
    Because walls can also be detected, this MUST be calibrated with
    the real gripper geometry.
  */

  gripperOpen();

  for (int i = 0; i < 20; i++) {
    float d = distanceFront();

    Serial.print("Objeto frente: ");
    Serial.println(d);

    if (d > 5 && d < 22) {
      stopMotors();

      while (distanceFront() > 8) {
        setMotorSpeed(85, 85);
        delay(40);
      }

      stopMotors();
      gripperClose();
      return true;
    }

    setMotorSpeed(90, 90);
    delay(80);
  }

  stopMotors();
  return false;
}


// ============================================================
// 18. DISPLAY / STARTUP
// ============================================================

void lcdMessage(const char* line1, const char* line2 = "") {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void startupDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CANDIDATES 2026");
  lcd.setCursor(0, 1);
  lcd.print(ACTIVE_TRACK == TRACK_A ? "PISTA A - MAZE" : "PISTA B - NIVELES");
  lcd.setCursor(0, 2);
  lcd.print("AUTO");
  lcd.setCursor(0, 3);
  lcd.print(CALIBRATION_MODE ? "CALIBRACION" : "COMPETENCIA");
}


// ============================================================
// 19. PISTA A - MAZE
// ============================================================

bool colorAlreadyCounted[4] = {false, false, false, false};

int colorIndex(ColorName c) {
  switch (c) {
    case COLOR_CYAN: return 0;
    case COLOR_YELLOW: return 1;
    case COLOR_ORANGE: return 2;
    case COLOR_PINK: return 3;
    default: return -1;
  }
}

void processMazeColor(ColorName c) {
  int idx = colorIndex(c);

  if (idx >= 0 && !colorAlreadyCounted[idx]) {
    colorAlreadyCounted[idx] = true;

    showColorOnLCD(c);

    Serial.print("COLOR DE PISTA A: ");
    Serial.println(colorName(c));

    delay(1000);
  }
}

bool mazeHasUnvisitedNeighbor() {
  return accessibleAndUnvisited(heading) ||
         accessibleAndUnvisited(rightOf(heading)) ||
         accessibleAndUnvisited(leftOf(heading)) ||
         accessibleAndUnvisited(opposite(heading));
}

Heading chooseMazeNeighbor() {
  /*
    DFS-style exploration:
    1. forward
    2. right
    3. left
    4. rear

    The map is built dynamically. No wall/color layout is preloaded.
  */

  if (accessibleAndUnvisited(heading)) return heading;
  if (accessibleAndUnvisited(rightOf(heading))) return rightOf(heading);
  if (accessibleAndUnvisited(leftOf(heading))) return leftOf(heading);
  if (accessibleAndUnvisited(opposite(heading))) return opposite(heading);

  return heading;
}

void runMaze() {
  lcdMessage("PISTA A", "Explorando");

  resetMap();

  int safetySteps = 0;
  const int MAX_STEPS = 160;

  while (safetySteps < MAX_STEPS) {
    safetySteps++;

    ColorName c = readColor();

    // Final checkpoint.
    if (c == COLOR_RED) {
      stopMotors();
      lcdMessage("CHECKPOINT FINAL", "PISTA A");
      delay(2000);
      return;
    }

    processMazeColor(c);

    measureWallsAtCurrentCell();

    // If every known route from this cell has been visited,
    // backtrack toward an accessible previously visited neighbor.
    Heading target = chooseMazeNeighbor();

    if (!accessibleAndUnvisited(target)) {
      // Backtracking search among neighboring cells.
      bool movedBack = false;

      Heading options[4] = {
        opposite(heading),
        leftOf(heading),
        rightOf(heading),
        heading
      };

      for (int i = 0; i < 4; i++) {
        Heading d = options[i];

        if (!mapGrid[robotX][robotY].knownWall[d]) continue;
        if (mapGrid[robotX][robotY].wall[d]) continue;

        int nx = robotX + dxForHeading(d);
        int ny = robotY + dyForHeading(d);

        if (!insideMap(nx, ny)) continue;

        if (mapGrid[nx][ny].visited) {
          moveToNeighbor(d);
          movedBack = true;
          break;
        }
      }

      if (!movedBack) {
        stopMotors();
        lcdMessage("MAZE SIN SALIDA", "REVISA CALIB.");
        return;
      }
    } else {
      moveToNeighbor(target);
    }
  }

  stopMotors();
  lcdMessage("LIMITE DE PASOS", "REVISA CALIB.");
}


// ============================================================
// 20. PISTA B - SECTION 1
// ============================================================

bool candidateBallCell() {
  int walls = countKnownWalls(robotX, robotY);

  /*
    The ball is placed in the central unit and that unit has three
    walls blocking entry, leaving one free side. This is the geometric
    clue used here.
  */
  return walls >= 3;
}

bool findBallInSection1() {
  resetMap();

  const int MAX_STEPS = 80;

  for (int step = 0; step < MAX_STEPS; step++) {

    ColorName c = readColor();

    if (c == COLOR_RED) {
      // We reached the checkpoint without finding the ball.
      stopMotors();
      lcdMessage("CHECKPOINT 1", "SIN PELOTA");
      return false;
    }

    measureWallsAtCurrentCell();

    if (candidateBallCell()) {
      stopMotors();
      lcdMessage("BUSCANDO PELOTA", "CELDA CANDIDATA");

      if (approachBallAndGrab()) {
        lcdMessage("PELOTA TOMADA", "GARAA CERRADA");
        delay(700);
        return true;
      }
    }

    Heading target = chooseMazeNeighbor();

    if (!accessibleAndUnvisited(target)) {
      bool moved = false;

      Heading options[4] = {
        opposite(heading),
        leftOf(heading),
        rightOf(heading),
        heading
      };

      for (int i = 0; i < 4; i++) {
        Heading d = options[i];

        if (!mapGrid[robotX][robotY].knownWall[d]) continue;
        if (mapGrid[robotX][robotY].wall[d]) continue;

        int nx = robotX + dxForHeading(d);
        int ny = robotY + dyForHeading(d);

        if (!insideMap(nx, ny)) continue;

        if (mapGrid[nx][ny].visited) {
          moveToNeighbor(d);
          moved = true;
          break;
        }
      }

      if (!moved) break;
    } else {
      moveToNeighbor(target);
    }
  }

  stopMotors();
  lcdMessage("NO SE ENCONTRO", "PELOTA");
  return false;
}


// ============================================================
// 21. PISTA B - FIND CHECKPOINT 1
// ============================================================

bool navigateToRedCheckpoint(int maxSteps) {
  for (int step = 0; step < maxSteps; step++) {
    ColorName c = readColor();

    if (c == COLOR_RED) {
      stopMotors();
      return true;
    }

    measureWallsAtCurrentCell();

    Heading target = chooseMazeNeighbor();

    if (accessibleAndUnvisited(target)) {
      moveToNeighbor(target);
      continue;
    }

    bool moved = false;

    Heading options[4] = {
      rightOf(heading),
      heading,
      leftOf(heading),
      opposite(heading)
    };

    for (int i = 0; i < 4; i++) {
      Heading d = options[i];

      if (!mapGrid[robotX][robotY].knownWall[d]) continue;
      if (mapGrid[robotX][robotY].wall[d]) continue;

      int nx = robotX + dxForHeading(d);
      int ny = robotY + dyForHeading(d);

      if (!insideMap(nx, ny)) continue;

      if (mapGrid[nx][ny].visited) {
        moveToNeighbor(d);
        moved = true;
        break;
      }
    }

    if (!moved) break;
  }

  stopMotors();
  return false;
}


// ============================================================
// 22. PISTA B - SECTION 2
// ============================================================

void runSection2() {
  lcdMessage("PISTA B", "SECCION 2");
  delay(500);

  /*
    The section is a 2x4 green grid with white lines.
    The robot should use the 3-channel TCRT5000 to avoid the white lines.
  */

  const unsigned long MAX_SECTION_TIME = 120000UL;
  unsigned long start = millis();

  while (millis() - start < MAX_SECTION_TIME) {

    ColorName c = readColor();

    // Red checkpoint.
    if (c == COLOR_RED) {
      stopMotors();
      lcdMessage("CHECKPOINT 2", "SECCION 2");
      delay(1500);
      return;
    }

    lineAvoidanceStep();
    delay(20);
  }

  stopMotors();
  lcdMessage("TIEMPO SECCION 2", "REVISA CALIB.");
}


// ============================================================
// 23. PISTA B - SECTION 3
// ============================================================

Heading directionFromTile(ColorName c) {
  /*
    Exact mapping from Reglamento, Imagen 2.6:
      CIAN    -> derecha
      AMARILLO-> izquierda
      NARANJA -> arriba
      ROSA    -> abajo

    Directions are ABSOLUTE TO THE TRACK, not relative to the robot.
  */

  if (c == COLOR_CYAN)   return EAST;
  if (c == COLOR_YELLOW) return WEST;
  if (c == COLOR_ORANGE) return NORTH;
  if (c == COLOR_PINK)   return SOUTH;

  return heading;
}

void runSection3() {
  lcdMessage("PISTA B", "SECCION 3");
  delay(700);

  const int MAX_TILES = 30;

  for (int i = 0; i < MAX_TILES; i++) {

    ColorName c = readColor();

    if (c == COLOR_GREEN) {
      stopMotors();
      lcdMessage("FIN", "PISTA B");
      delay(2500);
      return;
    }

    if (c == COLOR_RED) {
      // We may still be on the checkpoint transition.
      stopMotors();
      delay(300);
      continue;
    }

    if (c == COLOR_CYAN ||
        c == COLOR_YELLOW ||
        c == COLOR_ORANGE ||
        c == COLOR_PINK) {

      showColorOnLCD(c);

      Heading target = directionFromTile(c);

      faceHeading(target);
      moveOneCell();
      updateLogicalPosition(target);

      delay(150);
    } else {
      /*
        Unknown tile:
        move very slowly instead of making an arbitrary turn.
      */
      stopMotors();
      delay(120);
    }
  }

  stopMotors();
  lcdMessage("NO LLEGO A FIN", "REVISA CALIB.");
}


// ============================================================
// 24. PISTA B
// ============================================================

void runTrackB() {
  lcdMessage("PISTA B", "SECCION 1");
  delay(800);

  bool ballTaken = findBallInSection1();

  if (!ballTaken) {
    /*
      Even without the ball, continue toward checkpoint 1.
      This can still obtain the corresponding checkpoint points.
    */
    navigateToRedCheckpoint(60);
  } else {
    /*
      Continue searching until the red checkpoint.
    */
    navigateToRedCheckpoint(80);
  }

  // Section 2
  runSection2();

  // Section 3
  runSection3();
}


// ============================================================
// 25. CALIBRATION MODE
// ============================================================

void runCalibration() {
  stopMotors();

  lcdMessage("CALIBRACION", "SENSORES");

  while (true) {
    RGBSample c = readColorSample();

    Serial.print("TCS RGB=");
    Serial.print(c.r, 1);
    Serial.print(",");
    Serial.print(c.g, 1);
    Serial.print(",");
    Serial.print(c.b, 1);
    Serial.print(" H=");
    Serial.print(c.h, 1);
    Serial.print(" S=");
    Serial.print(c.s, 3);
    Serial.print(" V=");
    Serial.println(c.v, 1);

    Serial.print("ULTRA L=");
    Serial.print(distanceLeft());
    Serial.print(" F=");
    Serial.print(distanceFront());
    Serial.print(" R=");
    Serial.println(distanceRight());

    Serial.print("TCRT L=");
    Serial.print(digitalRead(LINE_LEFT));
    Serial.print(" C=");
    Serial.print(digitalRead(LINE_CENTER));
    Serial.print(" R=");
    Serial.println(digitalRead(LINE_RIGHT));

    Serial.println("--------------------------------");

    delay(500);
  }
}


// ============================================================
// 26. SETUP
// ============================================================

void setup() {

  Serial.begin(115200);
  delay(300);

  // Motors
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Ultrasonics
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  digitalWrite(TRIG_LEFT, LOW);
  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_RIGHT, LOW);

  // Line sensors
  pinMode(LINE_LEFT, INPUT);
  pinMode(LINE_CENTER, INPUT);
  pinMode(LINE_RIGHT, INPUT);

  // Extra IR sensors
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);

  // LCD
  lcd.init();
  lcd.backlight();

  // Servo
  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2500);
  gripperOpen();

  // PWM
  analogWriteFrequency(PIN_ENA, 20000);
  analogWriteFrequency(PIN_ENB, 20000);

  stopMotors();

  startupDisplay();

  // TCS34725
  if (!tcs.begin(TCS34725_ADDRESS, &Wire)) {
    Serial.println("ERROR: TCS34725 no encontrado.");
    lcdMessage("ERROR TCS34725", "REVISA I2C");
    delay(1500);
  } else {
    Serial.println("TCS34725 OK.");
  }

  Serial.println();
  Serial.println("================================");
  Serial.println(" CANDIDATES 2026");
  Serial.println(" ROBOT AUTONOMO");
  Serial.println("================================");
  Serial.print("PISTA: ");
  Serial.println(ACTIVE_TRACK == TRACK_A ? "A - MAZE" : "B - NIVELES");

  if (CALIBRATION_MODE) {
    runCalibration();
  }

  delay(1500);
}


// ============================================================
// 27. LOOP
// ============================================================

void loop() {

  stopMotors();

  if (ACTIVE_TRACK == TRACK_A) {
    runMaze();
  } else {
    runTrackB();
  }

  stopMotors();

  lcdMessage("RONDA TERMINADA", "MOTORES OFF");

  // Do not automatically restart the track.
  while (true) {
    stopMotors();
    delay(1000);
  }
}
