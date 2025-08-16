#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Preferences.h>

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

void onReceive(int numBytes) {
  // TODO: Parse I2C commands from master
}

void onRequest() {
  Wire.write((uint8_t*)&status, sizeof(status));
}

void setup() {
  Serial.begin(115200);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(34, 35); // A, B pins
  encoder.clearCount();

  // Stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  // I2C
  Wire.begin(0x10);  // I2C slave address
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // Load calibration
  prefs.begin("motor", false);
  calibrationOffset = prefs.getInt("cal_offset", 0);
  prefs.end();
}

void loop() {
  stepper.run();  // Run motor based on current target

  // Update status
  status.actual_position = encoder.getCount();
  status.target_position = targetPosition;
  status.calibration_offset = calibrationOffset;
  status.motor_state = 0; // TODO: implement states
  status.error_code = 0;  // TODO: implement errors
}
