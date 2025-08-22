/*
 * ==============================================================
 * Project:   AdriansNippleTwister (ESP32 Stepper Motor Controller I²C Slave)
 * File:      main.ino
 * Author:    Leon Reeh
 * Created:   16.08.2025
 * Platform:  ESP32 (Arduino Framework)
 *
 * Description:
 *  This firmware implements a stepper motor controller with
 *  encoder feedback, controlled over an I²C interface.
 *  The ESP32 acts as an I²C slave device and executes commands
 *  such as calibration, target positioning, stop, and reset.
 *
 * Features:
 *  - Open-loop motion with periodic encoder corrections
 *  - Stall detection for calibration and fault handling
 *  - Configurable I²C slave address
 *  - Status reporting via structured response
 *
 * Command Set (I²C):
 *  0x01 → Calibrate Motor
 *  0x02 → Set Target Position
 *  0x03 → Stop Motor
 *  0x04 → Reset Device
 *  0x05 → Get Status (returns struct)
 *
 * Status Struct:
 *  - actual_position
 *  - target_position
 *  - calibration_offset
 *  - motor_state
 *  - error_code
 *
 * License:   MIT
 * ==============================================================
 */


#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Preferences.h>

// ---------------------
// State Machine Masks
// ---------------------
#define SM_IDLE         0
#define SM_MOVING       1
#define SM_CALIBRATING  2
#define SM_ERROR        3
#define SM_STOP         4

// ---------------------
// Error Code Masks
// ---------------------
#define ERR_STALL         1
#define ERR_OUT_OF_RANGE  2
#define ERR_CAL           3
#define ERR_TBD1          4
#define ERR_TBD2          5
#define ERR_COM           6

// ---------------------
// I2C Adress Masks
// ---------------------
#define DEFAULT_I2C_ADDRESS 0x10
#define SECOND_I2C_ADDRESS  0x11
#define THIRD_I2C_ADDRESS   0x20
#define FORTH_I2C_ADDRESS   0x21

// ---------------------
// Preference Mode Masks
// ---------------------
#define RW_MODE false
#define RO_MODE true

// ---------------------
// EPROM Stored Data
// ---------------------
Preferences prefs;

// ---------------------
// Runtime Stored Data
// ---------------------
struct MotorStatus {
  int32_t actual_position;    // Encoder count
  int32_t target_position;    // Commanded target
  int32_t calibration_offset; // Zero reference
  uint16_t motor_state;       // 0=idle, 1=moving, 2=calibrating, 3= error, 4 = Stop
  uint16_t error_code;        // 0=ok, 1=stall, 2=out-of-range, 3= Calibration error, 4= tbd, 5= tbd, 6=comm error
} status;

// ---------------------
// Stepper setup
// ---------------------
AccelStepper stepper(AccelStepper::DRIVER, 23, 22);
ESP32Encoder encoder;

// ---------------------
// Globals setup
// ---------------------

//Motion Control 
static bool primed = false;
static int32_t lastTarget = 0;

// -------- Stall detection tuning --------
static const uint16_t STALL_WINDOW_MS            = 25;   // sampling window
static const int32_t  STALL_MIN_EXPECTED_STEPS   = 2;    // ignore tiny moves per window
static const float    STALL_MIN_RATIO            = 0.30; // encoder / expected (scaled), below -> suspect   (Increase to be stricter)
static const uint8_t  STALL_CONSECUTIVE_WINDOWS  = 6;    // how many bad windows in a row => stall (~150ms) (Increase to be less sensetive)
static const float    STALL_IGNORE_SPEED_STEPS_S = 20.0; // don’t check at very low speeds

// How many encoder counts correspond to ONE motor *step*.
// Set this to your mechanics (e.g. if encoder is on shaft: counts per step; if on output: include gearing).
static const float    ENC_COUNTS_PER_STEPPER_STEP = 1.0f; // <- (e.g., if your encoder yields 4 counts per motor step → set to 4.0f; if gear-down encoder, include the gear ratio).

// -------- Calibration / Homing tuning --------
static const bool     HOMING_TOWARD_MIN      = true;  // true = move in negative direction to hit the hard stop
static const int32_t  HOMING_FAST_MAX_SPEED  = 1200;  // steps/s during fast seek
static const int32_t  HOMING_SLOW_MAX_SPEED  = 300;   // steps/s during slow seek
static const int32_t  HOMING_ACCEL           = 400;   // steps/s^2 for both passes

static const int32_t  HOMING_BACKOFF_STEPS   = 400;   // steps to back off after first stall
static const uint32_t HOMING_STALL_SETTLE_MS = 150;   // pause between phases after stall
static const int32_t  HOMING_MAX_TRAVEL_FAST = 20000; // safety: max steps allowed in fast pass without stall
static const int32_t  HOMING_MAX_TRAVEL_SLOW = 4000;  // safety: max steps allowed in slow pass without stall
static const int32_t  HOME_OFFSET_STEPS      = 150;   // logical zero should be this many steps away from the hard stop

// -------- Encoder / Stepper tuning --------
#define ENC_A 34
#define ENC_B 35
static int32_t encoder_origin_offset = 0;
static int32_t MaxSpeed              = 1000;
static int32_t MaxAcceleration       = 200;
static const int32_t MAX_RANGE_STEPS = 12000;  // absolute max from logical zero (inclusive)

// ---------------------
// State Function Prototypes
// ---------------------
void startCalibration();
void idleMotor();
void moveMotor();
void stopMotor();

// ---------------------
// Transition Function Prototypes
// ---------------------
void setTarget(int32_t target);
bool setState(uint16_t code);
void setError(uint16_t code);
bool detectStall();

// ---------------------
// Helper functions
// ---------------------
void initStatus();
void initPreferences();
void updatePositionFromEncoder();
void deviceReset();
inline int32_t clampTargetToRange(int32_t tgt);

// ---------------------
//  I2C Interface
// ---------------------
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS; // Device Adress
uint8_t lastCommand = 0;

void onReceive(int numBytes) {
  if (numBytes < 1) return;  // Nothing to read

  lastCommand = Wire.read();
  uint8_t cmd = lastCommand;

  switch(cmd) {
    case 0x01: // Calibrate
      setState(SM_CALIBRATING);
      break;

    case 0x02: { // Set Target
      if (numBytes >= 5) {
        int32_t target = 0;
        for (int i = 0; i < 4; i++) target |= ((int32_t)Wire.read() & 0xFF) << (8 * i);
        // HARD REJECT out of bounds:
        if (target < 0 || target > MAX_RANGE_STEPS) {
           setError(ERR_OUT_OF_RANGE); 
           break; 
          }
        setTarget(target);
        setState(SM_MOVING);
        } else {
        setError(ERR_COM);
        }
        break;
}


    case 0x03: // Stop Motor
      stopMotor();
      setState(SM_STOP);
      break;

    case 0x04: // Reset Device
      deviceReset();
      break;

    case 0x05: // Get Status
      // handled in onRequest
      break;
    
    default:  // Unknown Command
      setError(ERR_COM);
      break;  
  }
}


void onRequest() {
  if (lastCommand == 0x05) {
    Wire.write((uint8_t*)&status, sizeof(status));
  }
}

// ---------------------
// Device setup
// ---------------------
void setup() {

  Serial.begin(115200);

  // Load Preferences & Status
  initPreferences();
  initStatus();

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(ENC_A, ENC_B); // A, B pins
  encoder.clearCount();

  // Stepper
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(MaxAcceleration);

  // I2C
 Wire.begin((int)i2cAddress);  // I2C slave address
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

}

// ---------------------
// Main Loop / State Machine
// ---------------------
void loop() {

  uint16_t state = status.motor_state;

  switch (state)
  {
  case SM_IDLE:
    idleMotor();
    break;

  case SM_MOVING:
    moveMotor();
    break;

  case SM_CALIBRATING:
    startCalibration();
    break;

  case SM_ERROR:
    stopMotor();
    break;

  case SM_STOP:
    stopMotor();
    break;

  default:
    break;
  }

}

// ---------------------
// State Function Implementations
// ---------------------
void idleMotor() {
  updatePositionFromEncoder();

  // Define acceptable tolerance in steps
  const int32_t tolerance = 5;  

  // Calculate error between actual and target
  int32_t error = status.target_position - status.actual_position;

  // If within tolerance → do nothing
  if (abs(error) <= tolerance) {
    return;
  }

  // If outside tolerance → correct position
  stepper.moveTo(status.target_position);
  stepper.run();

  // 🔹 Stall detection (placeholder for now)
  if (detectStall()) {
    setError(ERR_STALL);
    return;
  }

  // If correction completed → stay idle
  if (stepper.distanceToGo() == 0) {
    updatePositionFromEncoder();
      // Persist last known absolute position
    prefs.begin("motor", RW_MODE);
    prefs.putInt("last_pos", status.actual_position);
    prefs.end();

    setState(SM_IDLE);
    return;
  }
}

void startCalibration() {
  enum CalPhase : uint8_t {
    CAL_INIT = 0,
    CAL_FAST_SEEK,
    CAL_WAIT_AFTER_STALL1,
    CAL_BACKOFF,
    CAL_WAIT_AFTER_BACKOFF,
    CAL_SLOW_SEEK,
    CAL_WAIT_AFTER_STALL2,
    CAL_MOVE_TO_OFFSET,   
    CAL_SET_ZERO,         
    CAL_DONE,
    CAL_FAIL
  };

  static CalPhase   phase = CAL_INIT;
  static uint32_t   tMark = 0;
  static int32_t    startStepAtPhase = 0;
  static int32_t    dirSign = 0;
  static int32_t    savedMaxSpeed = 0;
  static int32_t    savedAccel    = 0;

  auto resetPhase = [&]() {
    phase = CAL_INIT;
    tMark = 0;
  };

  auto moveTowardDir = [&](int32_t farSteps) {
    int32_t target = stepper.currentPosition() + (dirSign * farSteps);
    stepper.moveTo(target);
  };

  updatePositionFromEncoder();

  switch (phase) {
    case CAL_INIT: {
      status.motor_state = SM_CALIBRATING;
      status.error_code  = 0;

      savedMaxSpeed = stepper.maxSpeed();
      savedAccel    = stepper.acceleration();

      dirSign = HOMING_TOWARD_MIN ? -1 : +1;

      stepper.setMaxSpeed(HOMING_FAST_MAX_SPEED);
      stepper.setAcceleration(HOMING_ACCEL);

      moveTowardDir(HOMING_MAX_TRAVEL_FAST * 2);
      startStepAtPhase = stepper.currentPosition();
      phase = CAL_FAST_SEEK;
      break;
    }

    case CAL_FAST_SEEK: {
      stepper.run();
      int32_t travel = abs(stepper.currentPosition() - startStepAtPhase);
      if (travel > HOMING_MAX_TRAVEL_FAST) { phase = CAL_FAIL; break; }
      if (detectStall()) { tMark = millis(); phase = CAL_WAIT_AFTER_STALL1; }
      break;
    }

    case CAL_WAIT_AFTER_STALL1: {
      if (millis() - tMark < HOMING_STALL_SETTLE_MS) { stepper.run(); break; }
      stepper.setMaxSpeed(HOMING_SLOW_MAX_SPEED);
      // Backoff opposite homing direction to unload the stop
      stepper.moveTo(stepper.currentPosition() + (-dirSign * HOMING_BACKOFF_STEPS));
      phase = CAL_BACKOFF;
      break;
    }

    case CAL_BACKOFF: {
      stepper.run();
      if (stepper.distanceToGo() == 0) { tMark = millis(); phase = CAL_WAIT_AFTER_BACKOFF; }
      break;
    }

    case CAL_WAIT_AFTER_BACKOFF: {
      if (millis() - tMark < HOMING_STALL_SETTLE_MS) { stepper.run(); break; }
      stepper.setMaxSpeed(HOMING_SLOW_MAX_SPEED);
      stepper.setAcceleration(HOMING_ACCEL);
      moveTowardDir(HOMING_MAX_TRAVEL_SLOW * 2);
      startStepAtPhase = stepper.currentPosition();
      phase = CAL_SLOW_SEEK;
      break;
    }

    case CAL_SLOW_SEEK: {
      stepper.run();
      int32_t travel = abs(stepper.currentPosition() - startStepAtPhase);
      if (travel > HOMING_MAX_TRAVEL_SLOW) { phase = CAL_FAIL; break; }
      if (detectStall()) { tMark = millis(); phase = CAL_WAIT_AFTER_STALL2; }
      break;
    }

    case CAL_WAIT_AFTER_STALL2: {
      if (millis() - tMark < HOMING_STALL_SETTLE_MS) { stepper.run(); break; }
      // NEW: Move away from the stop by HOME_OFFSET_STEPS and set zero THERE
      stepper.setMaxSpeed(HOMING_SLOW_MAX_SPEED);
      stepper.moveTo(stepper.currentPosition() + (-dirSign * HOME_OFFSET_STEPS));
      phase = CAL_MOVE_TO_OFFSET;
      break;
    }

    case CAL_MOVE_TO_OFFSET: {
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        phase = CAL_SET_ZERO;
      }
      break;
    }

    case CAL_SET_ZERO: {
      // Set logical zero here (HOME_OFFSET_STEPS away from the hard stop)
      encoder.clearCount();
      encoder_origin_offset = 0;
      status.actual_position = 0;
      status.target_position = 0;
      status.calibration_offset = 0;

      stepper.setCurrentPosition(0);

      // Persist "we are at logical zero"
      prefs.begin("motor", RW_MODE);
      prefs.putInt("cal_offset", status.calibration_offset); // remains 0
      prefs.putInt("last_pos", 0);                           // we are at zero now
      prefs.end();

      // Restore motion profile
      stepper.setMaxSpeed(savedMaxSpeed);
      stepper.setAcceleration(savedAccel);

      phase = CAL_DONE;
      break;
    }

    case CAL_DONE: {
      setState(SM_IDLE);
      resetPhase();
      return;
    }

    case CAL_FAIL: {
      stopMotor();
      setError(ERR_CAL);
      stepper.setMaxSpeed(savedMaxSpeed);
      stepper.setAcceleration(savedAccel);
      resetPhase();
      return;
    }
  }
}


void moveMotor() {

  int32_t target = status.target_position;

  // Only (re)prime the move when the target changes
  if (!primed || target != lastTarget) {
    stepper.moveTo(target);
    primed = true;
    lastTarget = target;
  }

  // Advance motion
  stepper.run();

  // Keep status in sync with reality
  updatePositionFromEncoder();

  // Stall detection hook (placeholder for now)
  if (detectStall()) {
    setError(ERR_STALL);
    return;
  }

  // Reached target? -> go idle
  if (stepper.distanceToGo() == 0) {
  // Persist last known absolute position
  prefs.begin("motor", RW_MODE);
  prefs.putInt("last_pos", status.actual_position);
  prefs.end();

  setState(SM_IDLE);
  return;
  }
}

void stopMotor() {
  stepper.stop();         // Stop movement as soon as possible
  stepper.setSpeed(0);    // Ensure speed is 0
  stepper.run();          // Process the stop
}

// ---------------------
// Transition Function Implementations
// ---------------------
void setTarget(int32_t target) {
  status.target_position = clampTargetToRange(target);
  status.motor_state = SM_MOVING;
}

bool detectStall() {
  // Don’t flag when we’re basically not moving or already at target
  if (stepper.distanceToGo() == 0) {
    // Reset internal counters and allow normal completion
    static uint8_t s_badWindows = 0;
    s_badWindows = 0;
    return false;
  }
  if (fabs(stepper.speed()) < STALL_IGNORE_SPEED_STEPS_S) {
    // Too slow to reliably judge movement—reset counters and skip
    static uint8_t s_badWindows = 0;
    s_badWindows = 0;
    return false;
  }

  // State across calls
  static uint32_t s_lastTs      = 0;
  static int32_t  s_lastStepPos = 0;
  static int32_t  s_lastEncPos  = 0;
  static uint8_t  s_badWindows  = 0;

  uint32_t now = millis();
  if (s_lastTs == 0) {
    // Prime on first call
    s_lastTs      = now;
    s_lastStepPos = stepper.currentPosition();
    s_lastEncPos  = status.actual_position; // ensure updatePositionFromEncoder() ran
    s_badWindows  = 0;
    return false;
  }

  // Only evaluate once per window
  if ((now - s_lastTs) < STALL_WINDOW_MS) return false;

  // Calculate deltas in this window
  int32_t stepNow = stepper.currentPosition();
  int32_t encNow  = status.actual_position;

  int32_t expectedSteps = abs(stepNow - s_lastStepPos);
  int32_t encCounts     = abs(encNow  - s_lastEncPos);

  // Move the window forward
  s_lastTs      = now;
  s_lastStepPos = stepNow;
  s_lastEncPos  = encNow;

  // Ignore tiny moves (accel/decel edges)
  if (expectedSteps < STALL_MIN_EXPECTED_STEPS) {
    s_badWindows = 0; // don’t penalize
    return false;
  }

  // Scale encoder counts to "equivalent motor steps" so we can compare apples-to-apples
  float encAsSteps = (ENC_COUNTS_PER_STEPPER_STEP > 0.0f)
                     ? (encCounts / ENC_COUNTS_PER_STEPPER_STEP)
                     : 0.0f;

  // Ratio of observed (encoder) to expected (stepper) movement
  float ratio = (expectedSteps > 0) ? (encAsSteps / (float)expectedSteps) : 1.0f;

  // Judge this window
  if (ratio < STALL_MIN_RATIO) {
    if (s_badWindows < 255) s_badWindows++;
  } else {
    // Healthy movement resets the counter
    s_badWindows = 0;
  }

  // Too many consecutive bad windows => stall
  if (s_badWindows >= STALL_CONSECUTIVE_WINDOWS) {
    // Reset for next time and report stall
    s_badWindows = 0;
    return true;
  }

  return false;
}

bool setState(uint16_t code){
  //Update motor state for state machine
  // Return true if OK return false if state is locked
  uint16_t cur_state = status.motor_state;
  if (cur_state == SM_ERROR)
  {
    return false; //error can only be cleared via deviceReset
  }
  else{
    status.motor_state = code; 
    return true;
  } 
}

void setError(uint16_t code) {
  status.error_code = code;
  status.motor_state = SM_ERROR; // error
}

// ---------------------
// Helper Implementations
// ---------------------
void initStatus() {
  prefs.begin("motor", RO_MODE);
  int32_t last_pos = prefs.getInt("last_pos", 0);   // absolute steps from logical zero
  status.actual_position    = last_pos;             // logical position
  status.target_position    = last_pos;             // keep target = where we are
  status.calibration_offset = prefs.getInt("cal_offset", 0); // stays 0 in this scheme
  status.motor_state        = SM_STOP;
  status.error_code         = 0;
  prefs.end();

  // Encoder is incremental and starts at 0 on boot.
  // Tie logical position to fresh encoder counts via an origin offset:
  encoder_origin_offset = last_pos;

  // Keep AccelStepper coordinate frame aligned with logical frame
  stepper.setCurrentPosition(last_pos);
}

void initPreferences(){
  prefs.begin("motor", RO_MODE);
  bool tpInit  = prefs.isKey("nvsInit"); 
  if (tpInit  == false) {
    prefs.end();
    prefs.begin("motor", RW_MODE);
    //initialize preference keys with "factory default" values on first startup.
    prefs.putUChar("i2c_addr", i2cAddress);
    prefs.putInt("cal_offset", 0);
    prefs.putInt("last_pos",0);
    prefs.putBool("nvsInit", true);          // Create the "already initialized Key"
    prefs.end();                             // Close the namespace in RW mode and...
    prefs.begin("motor", RO_MODE);        //  reopen it in RO mode
   }
  i2cAddress = prefs.getUChar("i2c_addr");
  prefs.end();
}

void updatePositionFromEncoder() {
  // logical position = offset at boot (or after homing) + live encoder counts
  status.actual_position = encoder_origin_offset + (int32_t)encoder.getCount();
}

void deviceReset(){
    initPreferences();
    initStatus();
}

inline int32_t clampTargetToRange(int32_t tgt) {
  const int32_t minBound = 0;                  // logical zero boundary
  // (Optional) add a maxBound if you have a physical max
  return (tgt < minBound) ? minBound : tgt;
}
