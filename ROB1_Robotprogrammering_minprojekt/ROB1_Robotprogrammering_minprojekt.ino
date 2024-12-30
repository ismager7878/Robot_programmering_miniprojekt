/*
 * read girscope data
 * Carlos Gomez Cubero
 * 
 * The gyroscope is calibrated at the beginning "turnSensorSetup();"
 * The calibration initialize the X, Y and Z axis of the girscopoe so place it 
 * in a flat surface.
 * 
 * When the yellow LED is off (few instants after start) you can press the A button to 
 * reset the rotation on the Z axis to zero and start measuring from there. The rotation
 * is sent thru the serial port.
 * 
 * 
 * Most of the code is extracted from pololu zumo32U4 example RotationResist
 * https://github.com/pololu/zumo-32u4-arduino-library/blob/master/examples/RotationResist/RotationResist.ino
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD oled;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

float revFactor = 1.03;
float factor = 1.015;

int lineSensorCalTries = 5;
int driveTime = 500;

#define NUMSENSORS 3

uint16_t lineSensorValues[NUMSENSORS];

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

int speed = 75;
int threshold = 200;

int lastDistance = 0;

char dir;

void setup() {
  Serial.begin(9600);
  turnSensorSetup();
  lineSensors.initThreeSensors();
  calibrateLineSensors();
  delay(500);
  turnSensorReset();
  oled.clear();
  findMaze();
}

void printToOLED(String a = "", String b = "") {
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print(a);
  oled.gotoXY(0, 1);
  oled.print(b);
}

void printReadingsToSerial() {
  printToOLED((String)lineSensorValues[0] + ", " + (String)lineSensorValues[1], (String)lineSensorValues[2]);
}

void calibrateLineSensors() {
  for (int i = 0; i < lineSensorCalTries; i++) {
    int lastTime = millis();
    motors.setSpeeds(speed, speed);
    while (millis() - lastTime < driveTime) {
      lineSensors.calibrate();
      delay(20);
    }
    speed = speed * -1;
  }
  stop();
  speed = abs(speed);

  String message[] = { "Cal", "Done", "Place at", "Maze and", "Press A" };
  awaitUser(message, 5);
}

bool lineDetected() {
  readLineSensors();
  return (lineSensorValues[0] > threshold || lineSensorValues[1] > threshold || lineSensorValues[2] > threshold);
}

void findMaze() {
  motors.setSpeeds(speed, speed);
  unsigned long startTime = millis();
  int time = random(2000, 4000);
  printToOLED("finding Maze");
  while ((millis() - startTime) < time) {
    updateMotorSpeeds();
    if (lineDetected()) {
      if (lineSensorValues[0] > threshold) {
        dir = 'l';
      } else {
        dir = 'r';
      }
      stop();
      return;
    }
  }
  turnByAngle(random(90, 361));
  delay(300);
  findMaze();
}

void turnByAngle(int angle) {
  printToOLED("Angle:", (String)angle);
  turnSensorReset();
  if (angle > 180) {
    motors.setSpeeds(speed, -speed);
    delay(20);
    while (getTurnAngleInDegrees() > angle) {
    }
  } else {
    motors.setSpeeds(-speed, speed);
    delay(20);
    while (getTurnAngleInDegrees() < angle) {
    }
  }
  stop();
  resetEncoders();
}

void stop() {
  motors.setSpeeds(0, 0);
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void awaitUser(String messages[], int len) {
  if (len > 2) {
    for (int i = 0; i < len; i++) {
      if (!(i + 1 < len)) {
        printToOLED(messages[i]);
      } else {
        printToOLED(messages[i], messages[i + 1]);
        i++;
      }
      unsigned long startTime = millis();
      while (millis() - startTime < 2000) {
        if (buttonA.getSingleDebouncedRelease()) {
          oled.clear();
          return;
        }
      }
      if (i + 1 == len) {
        i = -1;
      }
    }
  } else {
    printToOLED(messages[0], len == 2 ? messages[1] : "");
    while (!buttonA.getSingleDebouncedRelease()) {
    }
    oled.clear();
  }
}

void updateMotorSpeeds() {

  int countLeft = encoders.getCountsLeft();
  int countRight = encoders.getCountsRight();

  if (countLeft > 30000 || countRight > 30000) {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
  }

  int currentSpeedRight = speed + (countLeft - countRight * factor);
  motors.setSpeeds(speed, currentSpeedRight);
}

void readLineSensors() {
  lineSensors.readCalibrated(lineSensorValues, QTR_EMITTERS_ON);
  printReadingsToSerial();
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  return (((turnAngle >> 16) * 360) >> 16);
}

void loop() {
  if (lineDetected()) {
    if (lineSensorValues[0] > threshold) {
      if (dir == 'l') {
        turnByAngle(355);
      } else {
        turnByAngle(90);
      }
      motors.setSpeeds(-speed * 0.2, speed);
    } else {
      if (dir == 'r') {
        turnByAngle(5);
      } else {
        turnByAngle(270);
      }
      motors.setSpeeds(speed, -speed * 0.2);
    }
  }
}
/* This should be called in setup() to enable and calibrate the
gyro.  It uses the oled, yellow LED, and button A.  While the oled
is displaying "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  oled.clear();
  oled.print(F("Gyro cal"));

  // Turn on the yellow LED in case the oled is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  oled.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease()) {
    turnSensorUpdate();
    oled.gotoXY(0, 0);
    // do some math and pointer magic to turn angle in seconds to angle in degrees
    oled.print((((turnAngle >> 16) * 360) >> 16));
    oled.print(F("   "));
  }
  oled.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}