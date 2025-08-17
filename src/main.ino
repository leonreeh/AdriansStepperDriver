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

long targetPosition = 0;
long calibrationOffset = 0;

struct MotorStatus {
  int32_t actual_position;
  int32_t target_position;
  int32_t calibration_offset;
  uint16_t motor_state;
  uint16_t error_code;
} status;

/******************* I2C Interface ****************** */
void onReceive(int numBytes) {
  uint8_t cmd = Wire.read();
  
  switch(cmd) {
    case 0x01: // Calibrate
      startCalibration();
      break;

    case 0x02: // Set Target
      setTarget();
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
      //set error_code to communication error;
      //report error to Master
      break;  
  }
}

void onRequest() {
  if (lastCommand == 0x05) {
    Wire.write((uint8_t*)&status, sizeof(status));
  }
}
/******************* I2C Interface ****************** */

void setup() {
  Serial.begin(115200);

  // Load calibration
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
  calibrationOffset = prefs.getInt("cal_offset");
  prefs.end();

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
}
