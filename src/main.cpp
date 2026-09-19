
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>


#define TRACK_A 1
#define TRACK_B 2


#define ACTIVE_TRACK TRACK_A


#define CALIBRATION_MODE 0

#define PIN_ENA 14
#define PIN_IN1 27
#define PIN_IN2 25

#define PIN_ENB 32
#define PIN_IN3 26
#define PIN_IN4 33


#define INVERT_LEFT  false
#define INVERT_RIGHT false




#define TRIG_LEFT   18
#define ECHO_LEFT   34

#define TRIG_FRONT  19
#define ECHO_FRONT  36

#define TRIG_RIGHT  21
#define ECHO_RIGHT  35


const float WALL_DISTANCE_CM = 22.0;




#define I2C_SDA 22
#define I2C_SCL 23


#define LCD_ADDRESS 0x27

LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

Adafruit_TCS34725 tcs(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);



#define SERVO_PIN 5

Servo gripper;

const int SERVO_OPEN_ANGLE  = 55;
const int SERVO_CLOSE_ANGLE = 125;




#define LINE_LEFT   4
#define LINE_CENTER 16
#define LINE_RIGHT  17


#define LINE_WHITE_LEVEL LOW

int lastLineCorrection = 0; 



#define IR_1 15
#define IR_2 2



const int BASE_SPEED = 145;
const int TURN_SPEED = 145;

const unsigned long CELL_FORWARD_MS = 1450;
const unsigned long TURN_90_MS       = 430;

const unsigned long SHORT_FORWARD_MS = 250;



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


