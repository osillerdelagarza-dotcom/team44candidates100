#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>


// 
// Pista

#define TRACK_A 1
#define TRACK_B 2

#define ACTIVE_TRACK TRACK_A

#define CALIBRATION_MODE 0

// DRV8833 (por el momento)

#define PIN_IN1 25
#define PIN_IN2 26

// Puente H lado derecho
#define PIN_IN3 32
#define PIN_IN4 33


#define INVERT_LEFT  false
#define INVERT_RIGHT false


// Ultrasónicos


#define TRIG_LEFT   18
#define ECHO_LEFT   34

#define TRIG_FRONT  19
#define ECHO_FRONT  36

#define TRIG_RIGHT  21
#define ECHO_RIGHT  35

const float WALL_DISTANCE_CM = 22.0;

// I2C

#define I2C_SDA 23
#define I2C_SCL 22

// LCD 20x4 (parece ser la mejor opción)

#define LCD_ADDRESS 0x27

LiquidCrystal_I2C lcd(
    LCD_ADDRESS,
    20,
    4
);


// TCS34725


Adafruit_TCS34725 tcs(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);

// SERVO

#define SERVO_PIN 5

Servo gripper;

const int SERVO_OPEN_ANGLE  = 55;
const int SERVO_CLOSE_ANGLE = 125;

// Sensor líneas

#define LINE_LEFT   16
#define LINE_CENTER 17
#define LINE_RIGHT  27

#define LINE_WHITE_LEVEL LOW

int lastLineCorrection = 0;

// Sensores IR

#define IR_1 15
#define IR_2 4


// Velocidades


const int BASE_SPEED = 145;
const int TURN_SPEED = 145;

const unsigned long CELL_FORWARD_MS = 1450;
const unsigned long TURN_90_MS       = 430;

const unsigned long SHORT_FORWARD_MS = 250;



// COLORES


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



// Direcciones


enum Heading {

  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3

};



// MAPA


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



// CONTROL DEL MOTOR IZQUIERDO

void setLeftDirection(bool forward) {

  if (INVERT_LEFT) {
    forward = !forward;
  }

  if (forward) {

    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);

  }
  else {

    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);

  }

}



// CONTROL DEL MOTOR DERECHO


void setRightDirection(bool forward) {

  if (INVERT_RIGHT) {
    forward = !forward;
  }

  if (forward) {

    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);

  }
  else {

    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);

  }

}



// VELOCIDAD DE MOTORES

void setMotorSpeed(
    int leftSpeed,
    int rightSpeed
) {

  leftSpeed = constrain(
      leftSpeed,
      -255,
      255
  );

  rightSpeed = constrain(
      rightSpeed,
      -255,
      255
  );



  // MOTOR IZQUIERDO


  if (leftSpeed == 0) {

    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);

  }
  else {

    setLeftDirection(
        leftSpeed > 0
    );

    int speed =
        abs(leftSpeed);

    if (leftSpeed > 0) {

      analogWrite(
          PIN_IN1,
          speed
      );

      digitalWrite(
          PIN_IN2,
          LOW
      );

    }
    else {

      digitalWrite(
          PIN_IN1,
          LOW
      );

      analogWrite(
          PIN_IN2,
          speed
      );

    }

  }



  // MOTOR DERECHO


  if (rightSpeed == 0) {

    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);

  }
  else {

    setRightDirection(
        rightSpeed > 0
    );

    int speed =
        abs(rightSpeed);

    if (rightSpeed > 0) {

      analogWrite(
          PIN_IN3,
          speed
      );

      digitalWrite(
          PIN_IN4,
          LOW
      );

    }
    else {

      digitalWrite(
          PIN_IN3,
          LOW
      );

      analogWrite(
          PIN_IN4,
          speed
      );

    }

  }

}



// DETENER


void stopMotors() {

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);

}

// AVANZAR

void forward(int speed = BASE_SPEED) {

  setMotorSpeed(
      speed,
      speed
  );

}


// RETROCEDER


void backward(int speed = BASE_SPEED) {

  setMotorSpeed(
      -speed,
      -speed
  );

}

// GIRAR IZQUIERDA


void turnLeftInPlace(
    int speed = TURN_SPEED
) {

  setMotorSpeed(
      -speed,
      speed
  );

}


// GIRAR DERECHA


void turnRightInPlace(
    int speed = TURN_SPEED
) {

  setMotorSpeed(
      speed,
      -speed
  );

}



// AVANZAR UNA


void moveOneCell() {

  forward(BASE_SPEED);

  delay(
      CELL_FORWARD_MS
  );

  stopMotors();

  delay(80);

}



// AVANZAR POCO (no sabemos si lo dejaremos)



void moveShort() {

  forward(BASE_SPEED);

  delay(
      SHORT_FORWARD_MS
  );

  stopMotors();

}



// GIRO 90° IZQUIERDA

void turnLeft90() {

  turnLeftInPlace(
      TURN_SPEED
  );

  delay(
      TURN_90_MS
  );

  stopMotors();

  delay(80);

  heading =
      (Heading)(
          (heading + 3) % 4
      );

}



// GIRO 90° DERECHA


void turnRight90() {

  turnRightInPlace(
      TURN_SPEED
  );

  delay(
      TURN_90_MS
  );

  stopMotors();

  delay(80);

  heading =
      (Heading)(
          (heading + 1) % 4
      );

}



// GIRO 180°


void turnAround() {

  turnRight90();

  turnRight90();

}



// ORIENTAR ROBOT

void faceHeading(
    Heading target
) {

  int diff =
      (
        (int)target -
        (int)heading +
        4
      ) % 4;


  if (diff == 0) {

    return;

  }


  if (diff == 1) {

    turnRight90();

  }
  else if (diff == 2) {

    turnAround();

  }
  else if (diff == 3) {

    turnLeft90();

  }

}



// ULTRASONICO


float readDistanceCM(
    uint8_t trigPin,
    uint8_t echoPin
) {

  digitalWrite(
      trigPin,
      LOW
  );

  delayMicroseconds(3);


  digitalWrite(
      trigPin,
      HIGH
  );

  delayMicroseconds(10);

  digitalWrite(
      trigPin,
      LOW
  );


  unsigned long duration =
      pulseIn(
          echoPin,
          HIGH,
          30000UL
      );


  if (duration == 0) {

    return 400.0;

  }


  return (
      duration *
      0.0343f
  ) / 2.0f;

}


// DISTANCIA IZQUIERDA


float distanceLeft() {

  float d =
      readDistanceCM(
          TRIG_LEFT,
          ECHO_LEFT
      );

  delay(25);

  return d;

}



// DISTANCIA FRONTAL


float distanceFront() {

  float d =
      readDistanceCM(
          TRIG_FRONT,
          ECHO_FRONT
      );

  delay(25);

  return d;

}


// DISTANCIA DERECHA


float distanceRight() {

  float d =
      readDistanceCM(
          TRIG_RIGHT,
          ECHO_RIGHT
      );

  delay(25);

  return d;

}



// DETECCION DE PARED


bool wallAtRelativeLeft() {

  return
      distanceLeft()
      < WALL_DISTANCE_CM;

}


bool wallAtRelativeFront() {

  return
      distanceFront()
      < WALL_DISTANCE_CM;

}


bool wallAtRelativeRight() {

  return
      distanceRight()
      < WALL_DISTANCE_CM;

}


// FUNCIONES DEL MAPA


bool insideMap(
    int x,
    int y
) {

  return
      x >= 0 &&
      x < MAP_SIZE &&
      y >= 0 &&
      y < MAP_SIZE;

}


int dxForHeading(
    Heading h
) {

  if (h == EAST)
    return 1;

  if (h == WEST)
    return -1;

  return 0;

}


int dyForHeading(
    Heading h
) {

  if (h == NORTH)
    return 1;

  if (h == SOUTH)
    return -1;

  return 0;

}


Heading opposite(
    Heading h
) {

  return
      (Heading)(
          ((int)h + 2) % 4
      );

}


Heading leftOf(
    Heading h
) {

  return
      (Heading)(
          ((int)h + 3) % 4
      );

}


Heading rightOf(
    Heading h
) {

  return
      (Heading)(
          ((int)h + 1) % 4
      );

}


// REINICIAR MAPA


void resetMap() {

  for (
      int x = 0;
      x < MAP_SIZE;
      x++
  ) {

    for (
        int y = 0;
        y < MAP_SIZE;
        y++
    ) {

      mapGrid[x][y].visited =
          false;


      for (
          int d = 0;
          d < 4;
          d++
      ) {

        mapGrid[x][y].knownWall[d] =
            false;

        mapGrid[x][y].wall[d] =
            false;

      }

    }

  }


  robotX =
      MAP_CENTER;

  robotY =
      MAP_CENTER;

  heading =
      NORTH;

}


// REGISTRAR PARED


void markWall(
    int x,
    int y,
    Heading d,
    bool wall
) {

  if (!insideMap(x, y)) {

    return;

  }


  mapGrid[x][y]
      .knownWall[d] =
      true;

  mapGrid[x][y]
      .wall[d] =
      wall;


  int nx =
      x +
      dxForHeading(d);

  int ny =
      y +
      dyForHeading(d);


  if (insideMap(nx, ny)) {

    Heading od =
        opposite(d);


    mapGrid[nx][ny]
        .knownWall[od] =
        true;


    mapGrid[nx][ny]
        .wall[od] =
        wall;

  }

}


// MEDIR PAREDES DE LA CELDA


void measureWallsAtCurrentCell() {

  bool frontWall =
      wallAtRelativeFront();

  bool leftWall =
      wallAtRelativeLeft();

  bool rightWall =
      wallAtRelativeRight();


  Heading original =
      heading;


  turnAround();


  bool rearWall =
      wallAtRelativeFront();


  faceHeading(
      original
  );


  markWall(
      robotX,
      robotY,
      original,
      frontWall
  );


  markWall(
      robotX,
      robotY,
      leftOf(original),
      leftWall
  );


  markWall(
      robotX,
      robotY,
      rightOf(original),
      rightWall
  );


  markWall(
      robotX,
      robotY,
      opposite(original),
      rearWall
  );


  mapGrid[robotX][robotY]
      .visited = true;

}



// CONTAR PAREDES


int countKnownWalls(
    int x,
    int y
) {

  int count = 0;


  for (
      int d = 0;
      d < 4;
      d++
  ) {

    if (
        mapGrid[x][y].knownWall[d] &&
        mapGrid[x][y].wall[d]
    ) {

      count++;

    }

  }


  return count;

}


// CELDA ACCESIBLE Y NO VISITADA

bool accessibleAndUnvisited(
    Heading d
) {

  if (
      !mapGrid[robotX][robotY]
          .knownWall[d]
  ) {

    return false;

  }


  if (
      mapGrid[robotX][robotY]
          .wall[d]
  ) {

    return false;

  }


  int nx =
      robotX +
      dxForHeading(d);

  int ny =
      robotY +
      dyForHeading(d);


  if (
      !insideMap(nx, ny)
  ) {

    return false;

  }


  return
      !mapGrid[nx][ny]
          .visited;

}



// ACTUALIZAR POSICION LOGICA


void updateLogicalPosition(
    Heading movementDirection
) {

  robotX +=
      dxForHeading(
          movementDirection
      );

  robotY +=
      dyForHeading(
          movementDirection
      );


  if (
      !insideMap(
          robotX,
          robotY
      )
  ) {

    robotX =
        constrain(
            robotX,
            0,
            MAP_SIZE - 1
        );

    robotY =
        constrain(
            robotY,
            0,
            MAP_SIZE - 1
        );

  }

}


// MOVERSE A CELDA VECINA

void moveToNeighbor(
    Heading target
) {

  faceHeading(
      target
  );

  moveOneCell();

  updateLogicalPosition(
      target
  );

}

