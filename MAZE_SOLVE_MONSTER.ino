enum DetectedCase {

  CASE_STRAIGHT,
  CASE_LEFT_TURN,
  CASE_RIGHT_TURN,
  CASE_T_JUNCTION,
  CASE_CROSS_JUNCTION,
  CASE_DEAD_END
};

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
/*
 * PROJECT: 16-CHANNEL LINE FOLLOWER
 * BASE: PAPABOT junction and turn logic
 * ALGORITHM: Left-first exploration + shortest-path replay
 * BOARD: Arduino Nano / ATmega328P
 * MOTOR DRIVER: TB6612FNG
 *
 * 16-channel multiplexed sensor array.
 * No OLED or encoders.
 */

// ============================================================
// 1. HARDWARE CONFIGURATION
// ============================================================

// TB6612FNG - Motor A (Left)
#define AIN1 10
#define AIN2 8
#define PWMA 6

// TB6612FNG - Motor B (Right)
#define BIN1 11
#define BIN2 12
#define PWMB 9

#define STBY A3

// 16-channel multiplexer
#define s0 4
#define s1 5
#define s2 2
#define s3 3

#define SENSOR_PIN A2

// Buttons
#define CAL_BUTTON A0
#define START_BUTTON A1

// Status LED
#define STATUS_LED 13

// ============================================================
// 2. SENSOR CONFIGURATION
// ============================================================

bool isBlackLine = 0;

const byte NUM_SENSORS = 16;

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int threshold[NUM_SENSORS];

int sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];

int sensorWeight[NUM_SENSORS] = {
   7,  6,  5,  4,  3,  2,  1,  0,
   0, -1, -2, -3, -4, -5, -6, -7
};

int activeSensors = 0;
int onLine = 0;

float error = 0;

// ============================================================
// 3. PID CONFIGURATION
// ============================================================

float Kp = 0.06;
float Ki = 0.0;
float Kd = 1.0;

float P = 0;
float I = 0;
float D = 0;

float previousError = 0;
float PIDvalue = 0;

int currentSpeed = 30;
int lfSpeed = 120;

int leftSpeed = 0;
int rightSpeed = 0;

// ============================================================
// 4. JUNCTION AND TURN CONFIGURATION
// ============================================================

#define BASE_SPEED 120
#define TURN_SPEED 100
#define INCH_SPEED 100

#define INCH_TIME_MS 120

#define BLIND_SPIN_MS 120

#define TURN_TIMEOUT_MS 3000

#define JUNCTION_COOLDOWN_MS 300

#define DEAD_END_CONFIRM_MS 120

// Maze-end confirmation: at least this many of the 16 sensors must be active.
#define MAZE_END_ACTIVE_THRESHOLD 14

// Center sensors: 7 and 8
#define CENTER_LEFT 7
#define CENTER_RIGHT 8

// Left branch sensors: 0-3
#define LEFT_START 0
#define LEFT_END 3

// Right branch sensors: 12-15
#define RIGHT_START 12
#define RIGHT_END 15

// Center/forward sensors: 4-11
#define STRAIGHT_START 4
#define STRAIGHT_END 11

unsigned long lastJunctionTime = 0;
unsigned long lineLostTime = 0;

bool junctionHandled = false;

// ============================================================
// 4A. MAZE PATH STORAGE AND SOLVER STATE
// ============================================================

const byte MAX_PATH_LENGTH = 100;

char path[MAX_PATH_LENGTH];
byte pathLength = 0;
byte shortestPathIndex = 0;

bool pathSolved = false;
bool shortestPathRunMode = false;
bool waitingForShortestPath = false;
bool mazeFinished = false;

// ============================================================
// 5. MOTOR CONTROL
// ============================================================

// Positive speed = forward.
// Motor direction conventions retained from your original code.

void motor1run(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(PWMA, speed);

  }
  else if (speed < 0) {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    analogWrite(PWMA, -speed);

  }
  else {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);

    analogWrite(PWMA, 255);
  }
}

void motor2run(int speed) {

  speed = constrain(speed, -255, 255);

  if (speed > 0) {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMB, speed);

  }
  else if (speed < 0) {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(PWMB, -speed);

  }
  else {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMB, 255);
  }
}

void drive(int left, int right) {

  motor1run(left);
  motor2run(right);
}

void stopMotors() {

  drive(0, 0);
}

void rotateLeft() {

  drive(-TURN_SPEED, TURN_SPEED);
}

void rotateRight() {

  drive(TURN_SPEED, -TURN_SPEED);
}

void inchForward() {

  drive(INCH_SPEED, INCH_SPEED);

  delay(INCH_TIME_MS);

  stopMotors();
}

// ============================================================
// 6. SENSOR READING
// ============================================================

int sensorRead(int sensor) {

  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);

  return analogRead(SENSOR_PIN);
}

void readLine() {

  onLine = 0;

  for (byte i = 0; i < NUM_SENSORS; i++) {

    if (maxValues[i] <= minValues[i]) {
      sensorValue[i] = 0;
      sensorArray[i] = false;
      continue;
    }

    if (isBlackLine) {

      sensorValue[i] = map(
        sensorRead(i),
        minValues[i],
        maxValues[i],
        0,
        1000
      );

    }
    else {

      sensorValue[i] = map(
        sensorRead(i),
        minValues[i],
        maxValues[i],
        1000,
        0
      );
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);

    sensorArray[i] = (sensorValue[i] > 500);

    if (sensorArray[i]) {
      onLine = 1;
    }
  }
}

// ============================================================
// 7. SENSOR GROUP DETECTION
// ============================================================

int countActive(byte start, byte end) {

  int count = 0;

  for (byte i = start; i <= end; i++) {

    if (sensorArray[i]) {
      count++;
    }
  }

  return count;
}

bool centerOnBlack() {

  return (
    sensorArray[CENTER_LEFT] ||
    sensorArray[CENTER_RIGHT]
  );
}

bool leftBranchDetected() {

  return countActive(LEFT_START, LEFT_END) >= 1;
}

bool rightBranchDetected() {

  return countActive(RIGHT_START, RIGHT_END) >= 1;
}

bool straightDetected() {

  return countActive(STRAIGHT_START, STRAIGHT_END) >= 1;
}

// ============================================================
// 8. JUNCTION CLASSIFICATION
// ============================================================



DetectedCase detectCurrentCase() {

  readLine();

  bool left = leftBranchDetected();
  bool right = rightBranchDetected();
  bool straight = straightDetected();

  if (!onLine) {
    return CASE_DEAD_END;
  }

  if (left && right) {
    return CASE_CROSS_JUNCTION;
  }

  if (left && straight) {
    return CASE_T_JUNCTION;
  }

  if (right && straight) {
    return CASE_T_JUNCTION;
  }

  if (left) {
    return CASE_LEFT_TURN;
  }

  if (right) {
    return CASE_RIGHT_TURN;
  }

  return CASE_STRAIGHT;
}

bool isJunction(DetectedCase c) {

  return (
    c == CASE_LEFT_TURN ||
    c == CASE_RIGHT_TURN ||
    c == CASE_T_JUNCTION ||
    c == CASE_CROSS_JUNCTION
  );
}

// ============================================================
// 9. PID LINE FOLLOWING
// ============================================================

void resetPID() {

  P = 0;
  I = 0;
  D = 0;

  previousError = 0;
}

void linefollow() {

  error = 0;
  activeSensors = 0;

  for (byte i = 0; i < NUM_SENSORS; i++) {

    if (sensorArray[i]) {

      error += (
        sensorWeight[i] *
        sensorValue[i]
      );

      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    return;
  }

  error /= activeSensors;

  P = error;
  I += error;
  D = error - previousError;

  PIDvalue = (
    Kp * P +
    Ki * I +
    Kd * D
  );

  previousError = error;

  leftSpeed = currentSpeed - PIDvalue;
  rightSpeed = currentSpeed + PIDvalue;

  leftSpeed = constrain(leftSpeed, -100, 255);
  rightSpeed = constrain(rightSpeed, -100, 255);

  drive(leftSpeed, rightSpeed);
}

// ============================================================
// 10. TURN UNTIL CENTER SENSOR LOCKS ON LINE
// ============================================================

bool turnUntilCenterOnBlack(bool turnLeft) {

  unsigned long startTime = millis();

  bool centerLeftLine = false;

  if (turnLeft) {
    rotateLeft();
  }
  else {
    rotateRight();
  }

  // Move away from the existing line before searching.
  delay(BLIND_SPIN_MS);

  while (millis() - startTime < TURN_TIMEOUT_MS) {

    readLine();

    bool center = centerOnBlack();

    // First, center sensors must lose the old line.
    if (!center) {
      centerLeftLine = true;
    }

    // Then they must reacquire the new line.
    if (centerLeftLine && center) {

      stopMotors();

      delay(50);

      resetPID();

      return true;
    }

    if (turnLeft) {
      rotateLeft();
    }
    else {
      rotateRight();
    }
  }

  stopMotors();

  Serial.println(F("TURN TIMEOUT"));

  return false;
}

// ============================================================
// 11. JUNCTION HANDLING
// ============================================================

// ============================================================
// 11. MAZE PATH STORAGE AND SIMPLIFICATION
// ============================================================

void recordPath(char move) {

  if (pathLength < MAX_PATH_LENGTH - 1) {

    path[pathLength++] = move;
    path[pathLength] = '\0';

    Serial.print(F("Path recorded ["));
    Serial.print(pathLength);
    Serial.print(F("]: "));
    Serial.println(path);

  } else {

    Serial.println(F("PATH BUFFER FULL - cannot record more moves"));

  }
}


char getReplacement(char a, char b) {

  if (a == 'L' && b == 'L') return 'S';
  if (a == 'L' && b == 'S') return 'R';
  if (a == 'R' && b == 'L') return 'B';
  if (a == 'S' && b == 'L') return 'R';
  if (a == 'S' && b == 'S') return 'B';
  if (a == 'L' && b == 'R') return 'B';
  if (a == 'R' && b == 'S') return 'L';
  if (a == 'S' && b == 'R') return 'L';
  if (a == 'R' && b == 'R') return 'S';

  return 0;
}


bool simplifyPath() {

  if (pathLength == 0) {
    Serial.println(F("Cannot solve: path is empty"));
    return false;
  }

  Serial.print(F("Raw Dry-Run Path: "));
  Serial.println(path);

  bool changed = true;

  while (changed) {

    changed = false;

    for (byte i = 0; i + 2 < pathLength; i++) {

      if (path[i + 1] == 'B') {

        char replacement = getReplacement(path[i], path[i + 2]);

        if (replacement != 0) {

          path[i] = replacement;

          // Remove the two moves consumed by this simplification.
          for (byte j = i + 1; j + 2 < pathLength; j++) {
            path[j] = path[j + 2];
          }

          pathLength -= 2;
          path[pathLength] = '\0';

          changed = true;
          break;
        }
      }
    }
  }

  Serial.print(F("Calculated Shortest Path: "));
  Serial.println(path);

  // A remaining B means the route could not be fully simplified.
  for (byte i = 0; i < pathLength; i++) {

    if (path[i] == 'B') {
      Serial.println(F("Simplification incomplete: unresolved U-turn"));
      return false;
    }
  }

  return (pathLength > 0);
}


void resetMazePath() {

  pathLength = 0;
  shortestPathIndex = 0;

  pathSolved = false;
  shortestPathRunMode = false;
  waitingForShortestPath = false;
  mazeFinished = false;

  path[0] = '\0';
}


// ============================================================
// 12. MAZE END HANDLING
// ============================================================

void handleMazeEnd() {

  stopMotors();
  mazeFinished = true;

  Serial.println();
  Serial.println(F("========== MAZE END DETECTED =========="));

  // End of shortest-path replay.
  if (shortestPathRunMode) {

    shortestPathRunMode = false;
    waitingForShortestPath = false;

    digitalWrite(STATUS_LED, HIGH);

    Serial.println(F("SHORTEST PATH RUN FINISHED"));
    Serial.println(F("Robot stopped at maze end."));

    return;
  }

  // End of the initial left-first exploration (dry run).
  Serial.println(F("DRY RUN FINISHED. Simplifying path..."));

  pathSolved = simplifyPath();

  if (pathSolved) {

    digitalWrite(STATUS_LED, HIGH);

    Serial.println(F("SHORTEST PATH SOLVED"));
    Serial.print(F("Path: "));
    Serial.println(path);
    Serial.print(F("Length: "));
    Serial.println(pathLength);
    Serial.println(F("Press START_BUTTON (A1) to execute shortest path."));

    waitingForShortestPath = true;

  } else {

    digitalWrite(STATUS_LED, HIGH);

    Serial.println(F("SHORTEST PATH SOLVING FAILED"));
    Serial.println(F("Check path recording and maze-end detection."));

    waitingForShortestPath = false;
  }
}


// ============================================================
// 13. JUNCTION HANDLING + LEFT-FIRST SOLVER
// ============================================================

void handleJunction(DetectedCase c) {

  // Stop immediately at the junction.
  stopMotors();

  digitalWrite(STATUS_LED, HIGH);

  Serial.println(F("\\nJUNCTION DETECTED"));

  // STEP 1: Capture branches using the 16-sensor group definitions.
  bool leftAvailable = leftBranchDetected();
  bool rightAvailable = rightBranchDetected();
  bool straightAvailable = straightDetected();

  // STEP 2: Let the chassis settle while continuing to scan.
  unsigned long settleStart = millis();

  while (millis() - settleStart < 30) {

    readLine();

    if (leftBranchDetected()) leftAvailable = true;
    if (rightBranchDetected()) rightAvailable = true;
    if (straightDetected()) straightAvailable = true;
  }

  // STEP 3: Move forward to center the axle over the junction.
  if (c != CASE_DEAD_END) {

    unsigned long inchStart = millis();

    drive(INCH_SPEED, INCH_SPEED);

    while (millis() - inchStart < INCH_TIME_MS) {

      readLine();

      if (leftBranchDetected()) leftAvailable = true;
      if (rightBranchDetected()) rightAvailable = true;
      if (straightDetected()) straightAvailable = true;
    }

    stopMotors();
    delay(30);
  }

  // STEP 4: Recheck all branches after inching.
  readLine();

  if (leftBranchDetected()) leftAvailable = true;
  if (rightBranchDetected()) rightAvailable = true;
  if (straightDetected()) straightAvailable = true;

  // STEP 5: Detect the destination/end square.
  //
  // On the 5-array bot, the reference code used 11111 and confirmed it
  // after an additional 40 ms crawl. For this 16-array bot, use a
  // near-full-array threshold (14/16) and the same confirmation crawl.
  //
  // This confirmation is important because a normal wide junction
  // crossbar can briefly activate most/all sensors.
  if (countActive(0, NUM_SENSORS - 1) >= MAZE_END_ACTIVE_THRESHOLD) {

    Serial.println(F("Possible maze-end surface. Confirming..."));

    unsigned long confirmStart = millis();

    drive(INCH_SPEED, INCH_SPEED);

    while (millis() - confirmStart < 40) {
      readLine();
    }

    stopMotors();
    delay(25);
    readLine();

    if (countActive(0, NUM_SENSORS - 1) >= MAZE_END_ACTIVE_THRESHOLD) {

      Serial.println(F("MAZE END CONFIRMED"));
      handleMazeEnd();
      return;

    } else {

      Serial.println(F("Wide junction confirmed; continuing."));
    }
  }

  // STEP 6: Select the next move.
  Serial.print(F("LEFT: "));
  Serial.print(leftAvailable);

  Serial.print(F(" STRAIGHT: "));
  Serial.print(straightAvailable);

  Serial.print(F(" RIGHT: "));
  Serial.println(rightAvailable);

  // ----------------------------------------------------------
  // SHORTEST-PATH REPLAY MODE
  // ----------------------------------------------------------
  if (shortestPathRunMode) {

    if (shortestPathIndex >= pathLength) {

      Serial.println(F("ERROR: No remaining shortest-path move."));
      stopMotors();
      mazeFinished = true;
      digitalWrite(STATUS_LED, HIGH);
      return;
    }

    char nextAction = path[shortestPathIndex++];

    Serial.print(F("Shortest Path Step ["));
    Serial.print(shortestPathIndex);
    Serial.print(F("/"));
    Serial.print(pathLength);
    Serial.print(F("]: "));
    Serial.println(nextAction);

    if (nextAction == 'L') {

      Serial.println(F("FAST RUN: LEFT"));
      turnUntilCenterOnBlack(true);

    } else if (nextAction == 'R') {

      Serial.println(F("FAST RUN: RIGHT"));
      turnUntilCenterOnBlack(false);

    } else if (nextAction == 'S') {

      Serial.println(F("FAST RUN: STRAIGHT"));
      // The robot is already aligned with the straight path.

    } else {

      Serial.println(F("ERROR: Invalid move in simplified path."));
      stopMotors();
      mazeFinished = true;
      digitalWrite(STATUS_LED, HIGH);
      return;
    }

    resetPID();

    lastJunctionTime = millis();
    junctionHandled = true;

    digitalWrite(STATUS_LED, LOW);

    Serial.println(F("SHORTEST-PATH JUNCTION COMPLETE"));
    return;
  }

  // ----------------------------------------------------------
  // INITIAL DRY RUN: LEFT-FIRST EXPLORATION
  // Priority: LEFT -> STRAIGHT -> RIGHT -> U-TURN
  // Record the chosen decision at every junction/dead end.
  // ----------------------------------------------------------

  if (leftAvailable) {

    Serial.println(F("DRY RUN ACTION: LEFT"));
    recordPath('L');
    turnUntilCenterOnBlack(true);

  } else if (straightAvailable) {

    Serial.println(F("DRY RUN ACTION: STRAIGHT"));
    recordPath('S');
    // Continue forward; no turn required.

  } else if (rightAvailable) {

    Serial.println(F("DRY RUN ACTION: RIGHT"));
    recordPath('R');
    turnUntilCenterOnBlack(false);

  } else {

    Serial.println(F("DRY RUN ACTION: U-TURN"));
    recordPath('B');
    turnUntilCenterOnBlack(false);
  }

  resetPID();

  lastJunctionTime = millis();
  junctionHandled = true;

  digitalWrite(STATUS_LED, LOW);

  Serial.println(F("JUNCTION COMPLETE\\n"));
}


// ============================================================
// 14. CALIBRATION
// ============================================================

void calibrate() {

  Serial.println(F("CALIBRATION START"));

  stopMotors();

  digitalWrite(STATUS_LED, HIGH);

  for (byte i = 0; i < NUM_SENSORS; i++) {

    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  unsigned long startTime = millis();

  // Rotate the robot to expose all sensors to both surfaces.
  while (millis() - startTime < 3000) {

    motor1run(150);
    motor2run(-150);

    for (byte i = 0; i < NUM_SENSORS; i++) {

      int value = sensorRead(i);

      if (value < minValues[i]) {
        minValues[i] = value;
      }

      if (value > maxValues[i]) {
        maxValues[i] = value;
      }
    }
  }

  stopMotors();

  delay(100);

  for (byte i = 0; i < NUM_SENSORS; i++) {

    threshold[i] = (
      minValues[i] + maxValues[i]
    ) / 2;

    Serial.print(threshold[i]);
    Serial.print(' ');
  }

  Serial.println();

  digitalWrite(STATUS_LED, LOW);

  resetPID();

  Serial.println(F("CALIBRATION COMPLETE"));
}

// ============================================================
// 15. ROBOT EXECUTION
// ============================================================

void executeRobot() {

  if (mazeFinished || waitingForShortestPath) {
    stopMotors();
    return;
  }

  DetectedCase currentCase = detectCurrentCase();

  // Handle junctions during both exploration and shortest-path replay.
  // Dead ends are also decisions for the left-first solver because they
  // require a recorded U-turn.
  if (
    (isJunction(currentCase) || currentCase == CASE_DEAD_END) &&
    millis() - lastJunctionTime > JUNCTION_COOLDOWN_MS
  ) {

    if (currentCase == CASE_DEAD_END) {

      if (lineLostTime == 0) {
        lineLostTime = millis();
      }

      if (millis() - lineLostTime < DEAD_END_CONFIRM_MS) {
        return;
      }

    }

    handleJunction(currentCase);
    lineLostTime = 0;
    return;
  }

  if (currentCase != CASE_DEAD_END) {
    lineLostTime = 0;
  }

  // Normal line following.
  if (currentSpeed < lfSpeed) {
    currentSpeed++;
  }

  linefollow();
}


// ============================================================
// 16. SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  // Increase ADC sampling speed.
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Multiplexer pins
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  // Buttons
  pinMode(CAL_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);

  // LED
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(STATUS_LED, LOW);

  stopMotors();

  Serial.println(F("PAPABOT 16-CHANNEL EDITION"));

  // ----------------------------------------------------------
  // WAIT FOR CALIBRATION BUTTON
  // ----------------------------------------------------------

  Serial.println(F("Press A0 to calibrate"));

  while (digitalRead(CAL_BUTTON) == HIGH) {
    stopMotors();
  }

  delay(200);

  while (digitalRead(CAL_BUTTON) == LOW) {
    delay(5);
  }

  calibrate();

  // ----------------------------------------------------------
  // WAIT FOR START BUTTON
  // ----------------------------------------------------------

  Serial.println(F("Press A1 to start"));

  while (digitalRead(START_BUTTON) == HIGH) {
    stopMotors();
  }

  delay(200);

  while (digitalRead(START_BUTTON) == LOW) {
    delay(5);
  }

  currentSpeed = 30;

  resetMazePath();
  resetPID();

  lastJunctionTime = millis();

  Serial.println(F("ROBOT RUNNING - LEFT-FIRST DRY RUN"));
}

// ============================================================
// 17. MAIN LOOP
// ============================================================

void loop() {

  // After the dry run reaches the maze end and the path is solved,
  // wait for START_BUTTON (A1) to begin shortest-path replay.
  if (waitingForShortestPath) {

    stopMotors();

    if (digitalRead(START_BUTTON) == LOW) {

      delay(200);

      while (digitalRead(START_BUTTON) == LOW) {
        delay(5);
      }

      shortestPathIndex = 0;
      shortestPathRunMode = true;
      waitingForShortestPath = false;
      mazeFinished = false;

      currentSpeed = 30;
      resetPID();

      lastJunctionTime = millis();

      digitalWrite(STATUS_LED, LOW);

      Serial.println(F("STARTING SHORTEST-PATH EXECUTION"));
    }

    return;
  }

  // Emergency stop: either button, during robot movement.
  if (!mazeFinished && (
    digitalRead(CAL_BUTTON) == LOW ||
    digitalRead(START_BUTTON) == LOW
  )) {

    stopMotors();

    digitalWrite(STATUS_LED, HIGH);

    Serial.println(F("EMERGENCY STOP"));

    while (
      digitalRead(CAL_BUTTON) == LOW ||
      digitalRead(START_BUTTON) == LOW
    ) {
      delay(10);
    }

    while (true) {
      stopMotors();
    }
  }

  // Stay stopped after maze completion or failed path solving.
  if (mazeFinished) {
    stopMotors();
    return;
  }

  executeRobot();
}