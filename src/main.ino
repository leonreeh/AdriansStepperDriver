#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Preferences.h>

#define DEFAULT_I2C_ADDRESS 0x10
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;

// Stepper setup (stepPin, dirPin)
AccelStepper stepper(AccelStepper::DRIVER, 23, 22);
ESP32Encoder encoder;
Preferences prefs;

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
  prefs.begin("motor", false);
  i2cAddress = prefs.getUChar("i2c_addr", DEFAULT_I2C_ADDRESS);
  calibrationOffset = prefs.getInt("cal_offset", 0);
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
 //Loop
}
