#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Preferences.h>


/*EPROM Stored Data
*Keys:
* i2c_addr = device slave Adress
* cal_offset = offset for motor calibraton
* nvsInit = Initializer key
*/
Preferences prefs;
// Preference file Mode Masks
#define RW_MODE false
#define RO_MODE true

// I2C Adress
#define DEFAULT_I2C_ADDRESS 0x10
#define SECOND_I2C_ADDRESS  0x11
#define THIRD_I2C_ADDRESS   0x20
#define FORTH_I2C_ADDRESS   0x21
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;

// Stepper setup (stepPin, dirPin)
AccelStepper stepper(AccelStepper::DRIVER, 23, 22);
ESP32Encoder encoder;

struct MotorStatus {
  int32_t actual_position;
  int32_t target_position;
  int32_t calibration_offset;
  uint16_t motor_state;
  uint16_t error_code;
} status;

// ---------------------
// Function Prototypes
// ---------------------
void startCalibration();
void setTarget(int32_t target);
void stopMotor();
void deviceReset();

// ---------------------
// Helper functions
// ---------------------
void initStatus();
void initPreferences();
void moveMotorSteps(int32_t steps, bool direction);
bool detectStall();
void updatePositionFromEncoder();
void setError(uint16_t code);


// ---------------------
//  I2C Interface
// ---------------------
void onReceive(int numBytes) {
  uint8_t cmd = Wire.read();
  
  switch(cmd) {
    case 0x01: // Calibrate
      startCalibration();
      break;

    case 0x02: // Set Target
      setTarget(0);
      break;

    case 0x03: //Stop Motor
      stopMotor();
      break;

    case 0x04: //Reset Device
      deviceReset();
      break;

    case 0x05: // Get Status
      // handled in onRequest
      break;
    
    default:  // Unkown Command
      setError(6);
      break;  
  }
}

void onRequest() {
  if (lastCommand == 0x05) {
    Wire.write((uint8_t*)&status, sizeof(status));
  }
}


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
  Wire.begin(i2cAddress);  // I2C slave address
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

}

void loop() {
 /*
 * Check encoder Position
 * Correct if needed
 * wait for Instructions
 */
  updatePositionFromEncoder();
}

// ---------------------
// Function Implementations
// ---------------------
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

setTarget(int32_t target){
  /*
   * - Compare target with actual
   * - Check if movement crosses calibration limit
   * - Update struct
   * - Move motor to target
   * - Detect stall while moving
   * - Update actual_position or set error
   */
}

stopMotor(){

}

deviceReset(){
    initPreferences();
    initStatus();
}


// ---------------------
// Helper Implementations
// ---------------------
void initStatus() {
  prefs.begin("motor", RO_MODE);
  status.actual_position   = 0; //read encoder position
  status.target_position   = 0; //maybe add save function for last target
  status.calibration_offset = prefs.getInt("cal_offset");
  status.motor_state       = 0; // idle
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

void moveMotorSteps(int32_t steps, bool direction) {
  // 🔹 Placeholder for stepper control code
  // steps = number of steps to move
  // direction = true (CW), false (CCW)
}

bool detectStall() {
  // 🔹 Placeholder: check encoder delta vs expected
  // return true if stall detected
  return false;
}

void updatePositionFromEncoder() {
  // 🔹 Read encoder and update status.actual_position
}

void setError(uint16_t code) {
  status.error_code = code;
  status.motor_state = 3; // error
  Wire.write((uint8_t*)&status, sizeof(status));
}