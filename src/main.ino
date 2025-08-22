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

// ---------------------
// Helper functions
// ---------------------
void initStatus();
void initPreferences();
bool detectStall();
void updatePositionFromEncoder();
void setError(uint16_t code);
void deviceReset();


// ---------------------
//  I2C Interface
// ---------------------
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS; // Device Adress
uint8_t lastCommand = 0;

void onReceive(int numBytes) {
  lastCommand = Wire.read();
  
  switch(lastCommand) {
    case 0x01: // Calibrate
      setState(SM_CALIBRATING);
      break;

    case 0x02: // Set Target
      setTarget(0);
      setState(SM_MOVING);
      break;

    case 0x03: //Stop Motor
      stopMotor();
      setState(SM_STOP);
      break;

    case 0x04: //Reset Device
      deviceReset();
      break;

    case 0x05: // Get Status
      // handled in onRequest
      break;
    
    default:  // Unkown Command
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
  encoder.attachHalfQuad(34, 35); // A, B pins
  encoder.clearCount();

  // Stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

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
void idleMotor(){
  updatePositionFromEncoder();
  /* Function goal: incase the motor is moved by outside forces correct its position
  * - If actual positiom matches target position with in x tolereance do nothing
  * - if actual positiom mismatches target position outside of x tolereance correct position
  * - this function should rarely do anything
  */
}

void startCalibration(){
  /*
   * Implement calibration routine:
   * - set motor_state = calibrating
   * - perform stall detection loop
   * - validate multiple stall checks
   * - write calibration_offset
   * - return to idle
   */
}

void moveMotor(){
  int32_t target = status.target_position;
  // Tell AccelStepper to move relative to current position
  stepper.moveTo(target);
  // Run stepper toward target
  stepper.run();

  // Update encoder feedback
  updatePositionFromEncoder();

  // Check if we've arrived at target
  if (stepper.distanceToGo() == 0) {
    setState(SM_IDLE);
  }

  // 🔹 Stall detection and correction will go here later
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
  // Update status
  status.target_position = target;

  // Update state
  status.motor_state = SM_MOVING;
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

// ---------------------
// Helper Implementations
// ---------------------
void initStatus() {
  prefs.begin("motor", RO_MODE);
  status.actual_position   = 0; //read encoder position
  status.target_position   = 0; //maybe add save function for last target
  status.calibration_offset = prefs.getInt("cal_offset");
  status.motor_state       = SM_STOP; // idle
  status.error_code        = 0; // no error
  prefs.end();
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
    prefs.putBool("nvsInit", true);          // Create the "already initialized Key"
    prefs.end();                             // Close the namespace in RW mode and...
    prefs.begin("motor", RO_MODE);        //  reopen it in RO mode
   }
  i2cAddress = prefs.getUChar("i2c_addr");
  prefs.end();
}

bool detectStall() {
  // 🔹 Placeholder: check encoder delta vs expected
  // return true if stall detected
  return false;
}

void updatePositionFromEncoder() {
  status.actual_position = (int32_t)encoder.getCount();
}

void setError(uint16_t code) {
  status.error_code = code;
  status.motor_state = SM_ERROR; // error
}

void deviceReset(){
    initPreferences();
    initStatus();
}
