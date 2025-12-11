#include <Wire.h>
#include <MPU6050.h>
#include <SimpleFOC.h>

// -------- CONFIG --------

float TARGET_PITCH = 90.0;
float TARGET_ROLL = -13.0;
float TARGET_YAW = 0.0;

int PITCH_DIRECTION = -1;
int ROLL_DIRECTION = -1;
int YAW_DIRECTION = 1;

// homing params
// keping these slow for now ,, gimbal goes sometimes very crazy!---not quite stable yet
float HOMING_SPEED_MAX = 0.15;
float HOMING_SPEED_MIN = 0.05;
float HOMING_TOLERANCE = 3.0;    
float HOMING_SLOWDOWN = 25.0;    


unsigned long DIR_LOCK_TIME = 500;  // ms before allowing direction change
float DIR_CHANGE_THRESHOLD = 15.0;

float pitchKp = 0.12, pitchKi = 0.00, pitchKd = 0.015;
float rollKp = 0.12, rollKi = 0.00, rollKd = 0.015;
float yawKp = 0.08, yawKi = 0.00, yawKd = 0.01;

float MAX_STAB_SPEED = 1.2;
float DEADBAND = 0.8; 
float INTEGRAL_LIMIT = 5.0;

// complementary filter - 0.96 seems to work well
// higher = trust gyro more, lower = trust accel more
float ALPHA = 0.96;

float VOLTAGE_SUPPLY = 8.0;
float VOLTAGE_LIMIT = 4.0;

unsigned long PRINT_INTERVAL = 250;

// -------- HARDWARE --------

// two IMUs - camera one at default addr, body one with AD0 pulled high
MPU6050 cameraIMU(0x68);
MPU6050 bodyIMU(0x69);

BLDCMotor rollMotor = BLDCMotor(11);
BLDCMotor pitchMotor = BLDCMotor(11);
BLDCMotor yawMotor = BLDCMotor(11);

BLDCDriver3PWM rollDriver = BLDCDriver3PWM(2, 3, 4, 11);
BLDCDriver3PWM pitchDriver = BLDCDriver3PWM(5, 6, 7, 11);
BLDCDriver3PWM yawDriver = BLDCDriver3PWM(8, 9, 10, 11);

// -------- STATE --------

enum SystemState { STATE_INIT, STATE_HOMING, STATE_STABILIZING, STATE_CALIBRATING, STATE_STOPPED };
enum HomingPhase { HOME_ROLL, HOME_PITCH, HOME_COMPLETE };

SystemState currentState = STATE_INIT;
HomingPhase homingPhase = HOME_ROLL;

// angle tracking
float cameraPitch = 0, cameraRoll = 0, cameraYaw = 0;
float bodyPitch = 0, bodyRoll = 0, bodyYaw = 0;

// targets get locked after homing completes
float stabTargetPitch = TARGET_PITCH;
float stabTargetRoll = TARGET_ROLL;
float stabTargetYaw = 0;

// PID state
float pitchIntegral = 0, rollIntegral = 0, yawIntegral = 0;
float pitchPrevError = 0, rollPrevError = 0, yawPrevError = 0;

// raw IMU data
int16_t cax, cay, caz, cgx, cgy, cgz;
int16_t bax, bay, baz, bgx, bgy, bgz;

// calibration
float pitchCalOffset = 0;
float rollCalOffset = 0;

unsigned long prevLoopTime = 0;
unsigned long prevPrintTime = 0;
unsigned long homingStartTime = 0;

// homing needs to settle before moving on
int settleCounter = 0;
int SETTLE_COUNT = 10;


int rollLockedDir = 0;
int pitchLockedDir = 0;
unsigned long rollDirLockTime = 0;
unsigned long pitchDirLockTime = 0;

// simple moving average for smoothing
// tried kalman filter but this works fine and is simpler
#define FILTER_SIZE 5
float rollHistory[FILTER_SIZE] = {0};
float pitchHistory[FILTER_SIZE] = {0};
int filterIndex = 0;
float filteredRoll = 0;
float filteredPitch = 0;


float wrapAngle(float angle);
float shortestAngleDiff(float target, float current);
void updateAngleFilter();
float getFilteredRoll();
float getFilteredPitch();


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // fast i2c
  
  delay(1000);
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("    3-AXIS GIMBAL CONTROLLER"));
  Serial.println(F("========================================"));
  Serial.println();
  
  // init IMUs
  if (!setupIMUs()) {
    Serial.println(F("IMU init failed!"));
    currentState = STATE_STOPPED;
    return;
  }
  
  // init motors
  if (!setupMotors()) {
    Serial.println(F("Motor init failed!"));
    currentState = STATE_STOPPED;
    return;
  }
  
  initializeAngles();
  
  // print config
  Serial.println();
  Serial.println(F("Config:"));
  Serial.print(F("  Target Pitch: ")); Serial.print(TARGET_PITCH); Serial.println(F(" deg"));
  Serial.print(F("  Target Roll:  ")); Serial.print(TARGET_ROLL); Serial.println(F(" deg"));
  Serial.print(F("  Tolerance: ")); Serial.print(HOMING_TOLERANCE); Serial.println(F(" deg"));
  Serial.println();
  Serial.println(F("Commands: r=rehome s=status c=cal 0=stop"));
  Serial.println(F("          1/2/3=test motors q/w/e=flip dir"));
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("STARTING HOMING"));
  Serial.println(F("========================================"));
  Serial.println();
  
  // begin homing sequence
  currentState = STATE_HOMING;
  homingPhase = HOME_ROLL;
  homingStartTime = millis();
  settleCounter = 0;
  
  prevLoopTime = millis();
  prevPrintTime = millis();
}


void loop() {
  // calc dt
  unsigned long now = millis();
  float dt = (now - prevLoopTime) / 1000.0f;
  prevLoopTime = now;
  
  // clamp dt - things get weird if this is too big or small
  if (dt <= 0.0001f) dt = 0.001f;
  if (dt > 0.1f) dt = 0.1f;
  
  // always run FOC
  rollMotor.loopFOC();
  pitchMotor.loopFOC();
  yawMotor.loopFOC();
  
  // read sensors
  readIMUData();
  updateAngles(dt);
  
  // state machine
  switch (currentState) {
    case STATE_HOMING:
      executeHoming();
      break;
    case STATE_STABILIZING:
      executeStabilization(dt);
      break;
    case STATE_CALIBRATING:
      executeCalibration();
      break;
    case STATE_STOPPED:
      stopMotors();
      break;
    default:
      break;
  }
  
  processSerialInput();
  
  delayMicroseconds(500);
}

// ======== HOMING ========

/*
 * Homing logic
 * 
 * The basic idea: move each axis until it reaches target angle
 * Roll first, then pitch (yaw doesn't really need homing since no absolute ref)
 * 
 * Had issues with oscillation at first - the angle kept jumping across
 * the +/-180 boundary which made the motor reverse direction constantly.
 * Fixed by:
 * 1. using shortestAngleDiff() to handle wrapping
 * 2. adding direction locking so it cant reverse too quickly
 * 3. filtering the angle readings
 */
void executeHoming() {
  updateAngleFilter();
  
  float currentRoll = getFilteredRoll();
  float currentPitch = getFilteredPitch();
  
  // use shortest path - this handles the 180/-180 wraparound
  float rollError = shortestAngleDiff(TARGET_ROLL, currentRoll);
  float pitchError = shortestAngleDiff(TARGET_PITCH, currentPitch);
  
  // debug output
  if (millis() - prevPrintTime >= PRINT_INTERVAL) {
    Serial.print(F("HOMING ["));
    Serial.print(homingPhase == HOME_ROLL ? F("ROLL ") : F("PITCH"));
    Serial.print(F("] | Roll: "));
    Serial.print(currentRoll, 1);
    Serial.print(F(" (err:"));
    printSigned(rollError, 1);
    Serial.print(F(") | Pitch: "));
    Serial.print(currentPitch, 1);
    Serial.print(F(" (err:"));
    printSigned(pitchError, 1);
    Serial.print(F(") | Settle: "));
    Serial.print(settleCounter);
    Serial.print(F(" | Dir: "));
    Serial.println(homingPhase == HOME_ROLL ? rollLockedDir : pitchLockedDir);
    
    prevPrintTime = millis();
  }
  
  switch (homingPhase) {
    case HOME_ROLL:
      pitchMotor.move(0);
      yawMotor.move(0);
      
      if (abs(rollError) <= HOMING_TOLERANCE) {
        rollMotor.move(0);
        rollLockedDir = 0;
        settleCounter++;
        
        if (settleCounter >= SETTLE_COUNT) {
          Serial.println();
          Serial.println(F(">>> ROLL HOMED <<<"));
          Serial.println();
          homingPhase = HOME_PITCH;
          settleCounter = 0;
          pitchLockedDir = 0;
        }
      } else {
        settleCounter = 0;
        
        int desiredDir = (rollError > 0) ? 1 : -1;
        unsigned long now = millis();
        
        // direction locking logic
        if (rollLockedDir == 0) {
          rollLockedDir = desiredDir;
          rollDirLockTime = now;
        } else if (desiredDir != rollLockedDir) {
          // only allow direction change if enough time passed AND error is big enough
          bool canChange = (now - rollDirLockTime) > DIR_LOCK_TIME;
          canChange = canChange && (abs(rollError) > DIR_CHANGE_THRESHOLD);
          
          if (canChange) {
            rollLockedDir = desiredDir;
            rollDirLockTime = now;
            Serial.println(F("  [roll dir changed]"));
          }
        }
        
        float speed = calcHomingSpeed(abs(rollError));
        rollMotor.move(ROLL_DIRECTION * rollLockedDir * speed);
      }
      break;
      
    case HOME_PITCH:
      rollMotor.move(0);
      yawMotor.move(0);
      
      if (abs(pitchError) <= HOMING_TOLERANCE) {
        pitchMotor.move(0);
        pitchLockedDir = 0;
        settleCounter++;
        
        if (settleCounter >= SETTLE_COUNT) {
          Serial.println();
          Serial.println(F(">>> PITCH HOMED <<<"));
          Serial.println();
          homingPhase = HOME_COMPLETE;
        }
      } else {
        settleCounter = 0;
        
        int desiredDir = (pitchError > 0) ? 1 : -1;
        unsigned long now = millis();
        
        if (pitchLockedDir == 0) {
          pitchLockedDir = desiredDir;
          pitchDirLockTime = now;
        } else if (desiredDir != pitchLockedDir) {
          bool canChange = (now - pitchDirLockTime) > DIR_LOCK_TIME;
          canChange = canChange && (abs(pitchError) > DIR_CHANGE_THRESHOLD);
          
          if (canChange) {
            pitchLockedDir = desiredDir;
            pitchDirLockTime = now;
            Serial.println(F("  [pitch dir changed]"));
          }
        }
        
        float speed = calcHomingSpeed(abs(pitchError));
        pitchMotor.move(PITCH_DIRECTION * pitchLockedDir * speed);
      }
      break;
      
    case HOME_COMPLETE:
      finishHoming();
      break;
  }
  
  // timeout - 60 sec should be plenty
  if (millis() - homingStartTime > 60000) {
    Serial.println();
    Serial.println(F("!!! HOMING TIMEOUT !!!"));
    Serial.println(F("check motor directions and IMU orientation"));
    Serial.println();
    currentState = STATE_STOPPED;
    stopMotors();
  }
}

float calcHomingSpeed(float absError) {
  float speed;
  
  if (absError > HOMING_SLOWDOWN) {
    speed = HOMING_SPEED_MAX;
  } else {
    speed = HOMING_SPEED_MIN + (HOMING_SPEED_MAX - HOMING_SPEED_MIN) * (absError / HOMING_SLOWDOWN);
  }
  
  speed = constrain(speed, HOMING_SPEED_MIN, HOMING_SPEED_MAX);
  return speed;
}

void finishHoming() {
  stopMotors();
  delay(300);
  
  // lock in targets - use configured targets, not measured!
  stabTargetPitch = TARGET_PITCH;
  stabTargetRoll = TARGET_ROLL;
  stabTargetYaw = cameraYaw;  // no absolute ref for yaw
  
  // reset PID
  pitchIntegral = 0;
  rollIntegral = 0;
  yawIntegral = 0;
  pitchPrevError = 0;
  rollPrevError = 0;
  yawPrevError = 0;
  
  currentState = STATE_STABILIZING;
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("HOMING COMPLETE - STABILIZING"));
  Serial.println(F("========================================"));
  Serial.print(F("Targets: P=")); Serial.print(stabTargetPitch, 1);
  Serial.print(F(" R=")); Serial.print(stabTargetRoll, 1);
  Serial.print(F(" Y=")); Serial.println(stabTargetYaw, 1);
  Serial.println(F("========================================"));
  Serial.println();
}

// ======== STABILIZATION ========

/*
 * PID stabilization
 * 
 * pretty standard PID implementation
 * main things to note:
 * - anti-windup on integral term
 * - deadband to prevent jitter when close to target
 * - derivative on error (could do derivative on measurement instead)
 * 
 * TODO: maybe try cascaded PID later? inner loop on rate, outer on position
 */
void executeStabilization(float dt) {
  updateAngleFilter();
  
  float pitchError = shortestAngleDiff(stabTargetPitch, getFilteredPitch());
  float rollError = shortestAngleDiff(stabTargetRoll, getFilteredRoll());
  float yawError = shortestAngleDiff(stabTargetYaw, cameraYaw);
  
  // run PID for each axis
  float pitchOut = runPID(pitchError, pitchIntegral, pitchPrevError,
                          pitchKp, pitchKi, pitchKd, dt);
  float rollOut = runPID(rollError, rollIntegral, rollPrevError,
                         rollKp, rollKi, rollKd, dt);
  float yawOut = runPID(yawError, yawIntegral, yawPrevError,
                        yawKp, yawKi, yawKd, dt);
  
  if (abs(pitchError) < DEADBAND) {
    pitchOut = 0;
    pitchIntegral *= 0.9;
  }
  if (abs(rollError) < DEADBAND) {
    rollOut = 0;
    rollIntegral *= 0.9;
  }
  if (abs(yawError) < DEADBAND) {
    yawOut = 0;
    yawIntegral *= 0.9;
  }
  
  pitchMotor.move(PITCH_DIRECTION * pitchOut);
  rollMotor.move(ROLL_DIRECTION * rollOut);
  yawMotor.move(YAW_DIRECTION * yawOut);
  
  if (millis() - prevPrintTime >= PRINT_INTERVAL) {
    Serial.print(F("STAB | P:"));
    Serial.print(getFilteredPitch(), 1);
    Serial.print(F("("));
    printSigned(pitchError, 1);
    Serial.print(F(") R:"));
    Serial.print(getFilteredRoll(), 1);
    Serial.print(F("("));
    printSigned(rollError, 1);
    Serial.print(F(") Y:"));
    Serial.print(cameraYaw, 1);
    Serial.print(F("("));
    printSigned(yawError, 1);
    Serial.print(F(") | Out: "));
    printSigned(pitchOut, 2);
    Serial.print(F(" "));
    printSigned(rollOut, 2);
    Serial.print(F(" "));
    printSigned(yawOut, 2);
    Serial.println();
    
    prevPrintTime = millis();
  }
}

float runPID(float error, float& integral, float& prevError,
             float Kp, float Ki, float Kd, float dt) {
  // P
  float P = Kp * error;
  
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I = Ki * integral;
  
  float derivative = (error - prevError) / dt;
  float D = Kd * derivative;
  
  prevError = error;
  
  float output = P + I + D;
  output = constrain(output, -MAX_STAB_SPEED, MAX_STAB_SPEED);
  
  return output;
}


bool setupIMUs() {
  Serial.println(F("Init IMUs..."));
  
  cameraIMU.initialize();
  delay(50);
  if (!cameraIMU.testConnection()) {
    Serial.println(F("  Camera IMU FAIL"));
    return false;
  }
  Serial.println(F("  Camera IMU OK"));
  
  bodyIMU.initialize();
  delay(50);
  if (!bodyIMU.testConnection()) {
    Serial.println(F("  Body IMU FAIL"));
    return false;
  }
  Serial.println(F("  Body IMU OK"));
  
  // configure - using low range for better precision
  cameraIMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  cameraIMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  cameraIMU.setDLPFMode(MPU6050_DLPF_BW_20);
  
  bodyIMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  bodyIMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  bodyIMU.setDLPFMode(MPU6050_DLPF_BW_20);
  
  return true;
}

void readIMUData() {
  cameraIMU.getMotion6(&cax, &cay, &caz, &cgx, &cgy, &cgz);
  bodyIMU.getMotion6(&bax, &bay, &baz, &bgx, &bgy, &bgz);
}

void initializeAngles() {
  Serial.println(F("Init angle estimates..."));
  
  // take a bunch of samples to let things settle
  for (int i = 0; i < 100; i++) {
    readIMUData();
    delay(5);
  }
  
  cameraPitch = calcAccelPitch(cax, cay, caz) - pitchCalOffset;
  cameraRoll = calcAccelRoll(cay, caz) - rollCalOffset;
  cameraYaw = 0;
  
  bodyPitch = calcAccelPitch(bax, bay, baz);
  bodyRoll = calcAccelRoll(bay, baz);
  bodyYaw = 0;
  
  for (int i = 0; i < FILTER_SIZE; i++) {
    rollHistory[i] = cameraRoll;
    pitchHistory[i] = cameraPitch;
  }
  filteredRoll = cameraRoll;
  filteredPitch = cameraPitch;
  filterIndex = 0;
  
  Serial.print(F("  Initial: P="));
  Serial.print(cameraPitch, 1);
  Serial.print(F(" R="));
  Serial.println(cameraRoll, 1);
}

/*
 * complementary filter for angle estimation
 * 
 * gyro: good short term, drifts long term
 * accel: noisy but no drift
 * 
 * combine them: mostly trust gyro for quick changes,
 * slowly correct with accel to prevent drift
 */
void updateAngles(float dt) {
  // angles from accelerometer
  float camAccPitch = calcAccelPitch(cax, cay, caz) - pitchCalOffset;
  float camAccRoll = calcAccelRoll(cay, caz) - rollCalOffset;
  
  float bodyAccPitch = calcAccelPitch(bax, bay, baz);
  float bodyAccRoll = calcAccelRoll(bay, baz);
  
  float camGyroRoll = (float)cgx / 131.0f;
  float camGyroPitch = (float)cgy / 131.0f;
  float camGyroYaw = (float)cgz / 131.0f;
  
  float bodyGyroRoll = (float)bgx / 131.0f;
  float bodyGyroPitch = (float)bgy / 131.0f;
  float bodyGyroYaw = (float)bgz / 131.0f;
  
  // complementary filter
  cameraPitch = ALPHA * (cameraPitch + camGyroPitch * dt) + (1.0f - ALPHA) * camAccPitch;
  cameraRoll = ALPHA * (cameraRoll + camGyroRoll * dt) + (1.0f - ALPHA) * camAccRoll;
  cameraYaw += camGyroYaw * dt;  // yaw just integrates gyro, will drift
  
  bodyPitch = ALPHA * (bodyPitch + bodyGyroPitch * dt) + (1.0f - ALPHA) * bodyAccPitch;
  bodyRoll = ALPHA * (bodyRoll + bodyGyroRoll * dt) + (1.0f - ALPHA) * bodyAccRoll;
  bodyYaw += bodyGyroYaw * dt;
  
  // keep yaw bounded
  cameraYaw = wrapAngle(cameraYaw);
  bodyYaw = wrapAngle(bodyYaw);
}

float calcAccelPitch(int16_t ax, int16_t ay, int16_t az) {
  return atan2((float)ax, sqrt((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
}

float calcAccelRoll(int16_t ay, int16_t az) {
  return atan2((float)ay, (float)az) * RAD_TO_DEG;
}


bool setupMotors() {
  Serial.println(F("Init motors..."));
  
  rollDriver.voltage_power_supply = VOLTAGE_SUPPLY;
  rollDriver.init();
  rollMotor.linkDriver(&rollDriver);
  rollMotor.voltage_limit = VOLTAGE_LIMIT;
  rollMotor.controller = MotionControlType::velocity_openloop;
  rollMotor.init();
  rollMotor.enable();
  Serial.println(F("  Roll OK"));
  delay(50);
  
  // pitch
  pitchDriver.voltage_power_supply = VOLTAGE_SUPPLY;
  pitchDriver.init();
  pitchMotor.linkDriver(&pitchDriver);
  pitchMotor.voltage_limit = VOLTAGE_LIMIT;
  pitchMotor.controller = MotionControlType::velocity_openloop;
  pitchMotor.init();
  pitchMotor.enable();
  Serial.println(F("  Pitch OK"));
  delay(50);
  
  // yaw
  yawDriver.voltage_power_supply = VOLTAGE_SUPPLY;
  yawDriver.init();
  yawMotor.linkDriver(&yawDriver);
  yawMotor.voltage_limit = VOLTAGE_LIMIT;
  yawMotor.controller = MotionControlType::velocity_openloop;
  yawMotor.init();
  yawMotor.enable();
  Serial.println(F("  Yaw OK"));
  
  return true;
}

void stopMotors() {
  rollMotor.move(0);
  pitchMotor.move(0);
  yawMotor.move(0);
}

// ======== CALIBRATION ========

// simple calibration - just averages readings while stationary
// could probably do better but this works for now
void executeCalibration() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("CALIBRATING - KEEP STILL!"));
  Serial.println(F("========================================"));
  
  stopMotors();
  delay(500);
  
  int samples = 200;
  float pitchSum = 0, rollSum = 0;
  
  for (int i = 0; i < samples; i++) {
    readIMUData();
    
    pitchSum += calcAccelPitch(cax, cay, caz);
    rollSum += calcAccelRoll(cay, caz);
    
    if (i % 40 == 0) {
      Serial.print(F("  "));
      Serial.print((i * 100) / samples);
      Serial.println(F("%"));
    }
    delay(10);
  }
  
  pitchCalOffset = pitchSum / samples;
  rollCalOffset = rollSum / samples;
  
  Serial.println();
  Serial.println(F("Calibration done"));
  Serial.print(F("  Pitch offset: ")); Serial.println(pitchCalOffset, 2);
  Serial.print(F("  Roll offset:  ")); Serial.println(rollCalOffset, 2);
  Serial.println(F("========================================"));
  Serial.println();
  
  initializeAngles();
  currentState = STATE_STABILIZING;
}

// ======== SERIAL COMMANDS ========

void processSerialInput() {
  if (!Serial.available()) return;
  
  char cmd = Serial.read();
  
  switch (cmd) {
    case 'r':
    case 'R':
      Serial.println(F("\n>>> REHOMING <<<\n"));
      currentState = STATE_HOMING;
      homingPhase = HOME_ROLL;
      homingStartTime = millis();
      settleCounter = 0;
      rollLockedDir = 0;
      pitchLockedDir = 0;
      break;
      
    case 's':
    case 'S':
      printStatus();
      break;
      
    case 'c':
    case 'C':
      currentState = STATE_CALIBRATING;
      break;
      
    case 'p':
    case 'P':
      printPID();
      break;
      
    case '+':
      pitchKp += 0.02f;
      rollKp += 0.02f;
      Serial.print(F("Kp = ")); Serial.println(pitchKp, 3);
      break;
      
    case '-':
      pitchKp = max(0.0f, pitchKp - 0.02f);
      rollKp = max(0.0f, rollKp - 0.02f);
      Serial.print(F("Kp = ")); Serial.println(pitchKp, 3);
      break;
      
    case 'i':
      pitchKi += 0.005f;
      rollKi += 0.005f;
      Serial.print(F("Ki = ")); Serial.println(pitchKi, 4);
      break;
      
    case 'I':
      pitchKi = max(0.0f, pitchKi - 0.005f);
      rollKi = max(0.0f, rollKi - 0.005f);
      Serial.print(F("Ki = ")); Serial.println(pitchKi, 4);
      break;
      
    case 'd':
      pitchKd += 0.005f;
      rollKd += 0.005f;
      Serial.print(F("Kd = ")); Serial.println(pitchKd, 4);
      break;
      
    case 'D':
      pitchKd = max(0.0f, pitchKd - 0.005f);
      rollKd = max(0.0f, rollKd - 0.005f);
      Serial.print(F("Kd = ")); Serial.println(pitchKd, 4);
      break;
      
    case '0':
      Serial.println(F("\n!!! STOPPED !!!\n"));
      currentState = STATE_STOPPED;
      stopMotors();
      break;
      
    case 'g':
    case 'G':
      if (currentState == STATE_STOPPED) {
        Serial.println(F("\n>>> RESUMING <<<\n"));
        currentState = STATE_HOMING;
        homingPhase = HOME_ROLL;
        homingStartTime = millis();
        settleCounter = 0;
        rollLockedDir = 0;
        pitchLockedDir = 0;
      }
      break;
      
    case '1':
      testMotor("ROLL", rollMotor, cameraRoll);
      break;
      
    case '2':
      testMotor("PITCH", pitchMotor, cameraPitch);
      break;
      
    case '3':
      testMotor("YAW", yawMotor, cameraYaw);
      break;
      
    case 'q':
    case 'Q':
      ROLL_DIRECTION *= -1;
      Serial.print(F("Roll dir = ")); Serial.println(ROLL_DIRECTION);
      break;
      
    case 'w':
    case 'W':
      PITCH_DIRECTION *= -1;
      Serial.print(F("Pitch dir = ")); Serial.println(PITCH_DIRECTION);
      break;
      
    case 'e':
    case 'E':
      YAW_DIRECTION *= -1;
      Serial.print(F("Yaw dir = ")); Serial.println(YAW_DIRECTION);
      break;
  }
}

// run motor for 3 sec and show angle change
void testMotor(const char* name, BLDCMotor& motor, float& angle) {
  Serial.println();
  Serial.print(F("=== TESTING ")); Serial.print(name); Serial.println(F(" ==="));
  Serial.print(F("Start angle: ")); Serial.println(angle, 1);
  
  float startAngle = angle;
  unsigned long testStart = millis();
  
  while (millis() - testStart < 3000) {
    rollMotor.loopFOC();
    pitchMotor.loopFOC();
    yawMotor.loopFOC();
    readIMUData();
    updateAngles(0.01);
    motor.move(0.2);
    
    if ((millis() - testStart) % 500 < 10) {
      Serial.print(F("  ")); Serial.println(angle, 1);
    }
    delay(2);
  }
  
  motor.move(0);
  
  float change = angle - startAngle;
  Serial.print(F("End angle: ")); Serial.println(angle, 1);
  Serial.print(F("Change: ")); Serial.println(change, 1);
  Serial.println();
  
  if (change > 0) {
    Serial.println(F("Angle INCREASED -> direction should be +1"));
  } else {
    Serial.println(F("Angle DECREASED -> direction should be -1"));
  }
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("STATUS"));
  Serial.println(F("========================================"));
  
  Serial.print(F("State: "));
  switch (currentState) {
    case STATE_INIT:        Serial.println(F("INIT")); break;
    case STATE_HOMING:      Serial.println(F("HOMING")); break;
    case STATE_STABILIZING: Serial.println(F("STABILIZING")); break;
    case STATE_CALIBRATING: Serial.println(F("CALIBRATING")); break;
    case STATE_STOPPED:     Serial.println(F("STOPPED")); break;
  }
  
  Serial.println();
  Serial.println(F("Camera IMU:"));
  Serial.print(F("  P=")); Serial.print(cameraPitch, 1);
  Serial.print(F(" R=")); Serial.print(cameraRoll, 1);
  Serial.print(F(" Y=")); Serial.println(cameraYaw, 1);
  
  Serial.println(F("Body IMU:"));
  Serial.print(F("  P=")); Serial.print(bodyPitch, 1);
  Serial.print(F(" R=")); Serial.print(bodyRoll, 1);
  Serial.print(F(" Y=")); Serial.println(bodyYaw, 1);
  
  Serial.println();
  Serial.println(F("Targets:"));
  Serial.print(F("  P=")); Serial.print(stabTargetPitch, 1);
  Serial.print(F(" R=")); Serial.print(stabTargetRoll, 1);
  Serial.print(F(" Y=")); Serial.println(stabTargetYaw, 1);
  
  Serial.println();
  Serial.println(F("Motor directions:"));
  Serial.print(F("  Roll=")); Serial.print(ROLL_DIRECTION);
  Serial.print(F(" Pitch=")); Serial.print(PITCH_DIRECTION);
  Serial.print(F(" Yaw=")); Serial.println(YAW_DIRECTION);
  
  Serial.println(F("========================================"));
  Serial.println();
}

void printPID() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("PID VALUES"));
  Serial.println(F("========================================"));
  Serial.print(F("Pitch: Kp=")); Serial.print(pitchKp, 3);
  Serial.print(F(" Ki=")); Serial.print(pitchKi, 4);
  Serial.print(F(" Kd=")); Serial.println(pitchKd, 4);
  
  Serial.print(F("Roll:  Kp=")); Serial.print(rollKp, 3);
  Serial.print(F(" Ki=")); Serial.print(rollKi, 4);
  Serial.print(F(" Kd=")); Serial.println(rollKd, 4);
  
  Serial.print(F("Yaw:   Kp=")); Serial.print(yawKp, 3);
  Serial.print(F(" Ki=")); Serial.print(yawKi, 4);
  Serial.print(F(" Kd=")); Serial.println(yawKd, 4);
  Serial.println(F("========================================"));
  Serial.println();
}

  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// returns shortest angular distance from current to target
// handles the discontinuity at +/-180
float shortestAngleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

// moving average with wraparound handling
void updateAngleFilter() {
  rollHistory[filterIndex] = cameraRoll;
  pitchHistory[filterIndex] = cameraPitch;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  
  // for roll we need to handle wraparound in the average
  // use first sample as reference point
  float sumRoll = 0, sumPitch = 0;
  float refRoll = rollHistory[0];
  
  for (int i = 0; i < FILTER_SIZE; i++) {
    float relRoll = rollHistory[i] - refRoll;
    while (relRoll > 180.0f) relRoll -= 360.0f;
    while (relRoll < -180.0f) relRoll += 360.0f;
    sumRoll += relRoll;
    
    sumPitch += pitchHistory[i];
  }
  
  filteredRoll = wrapAngle(refRoll + sumRoll / FILTER_SIZE);
  filteredPitch = sumPitch / FILTER_SIZE;
}

float getFilteredRoll() { return filteredRoll; }
float getFilteredPitch() { return filteredPitch; }

void printSigned(float val, int dec) {
  if (val >= 0) Serial.print(F("+"));
  Serial.print(val, dec);
}
