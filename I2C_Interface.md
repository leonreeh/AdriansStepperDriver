# I²C Interface & Register Map

## 📌 Overview
The ESP32 stepper motor controller communicates with a master device over **I²C** in **slave mode**.  
Commands are issued from the master and processed by the ESP32; responses (such as status) are provided on request.  

### Default Slave Address
```cpp
#define DEFAULT_I2C_ADDRESS 0x10
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;
```
The I²C address is configurable in firmware for multiple devices.

---

## ⚙️ Command Set

### 0x01 – CALIBRATE
- **Direction**: Master → Slave  
- **Payload**: none  
- **Action**: Starts calibration sequence (stall-detection homing).  
- **Response**: none  

---

### 0x02 – SET_TARGET
- **Direction**: Master → Slave  
- **Payload**: 4 bytes (signed 32-bit int, little endian)  
  - New absolute target position in encoder counts.  
- **Action**: Motor moves to target position using open-loop motion with encoder correction.  
- **Response**: none  

---

### 0x03 – STOP MOTOR
- **Direction**: Master → Slave  
- **Payload**: none  
- **Action**: Immediately stops motor motion.  
- **Response**: none  

---

### 0x04 – RESET DEVICE
- **Direction**: Master → Slave  
- **Payload**: none  
- **Action**: Resets device state and clears errors.  
- **Response**: none  

---

### 0x05 – GET STATUS
- **Direction**: Master → Slave  
- **Payload**: none  
- **Response**: Returns the current motor status structure.  

```c
struct MotorStatus {
    int32_t actual_position;     // Encoder count
    int32_t target_position;     // Commanded target
    int32_t calibration_offset;  // Zero reference
    uint16_t motor_state;        // 0=idle, 1=moving, 2=calibrating, 3= error, 4= stop
    uint16_t error_code;         // 0=ok, 1=stall, 2=out-of-range, 3= Calibration error, 4= tbd, 5= tbd, 6=comm error
};
```
**Total size:** 16 bytes  

---

## 🛠️ Error Handling
If an **unknown command** is received:  
- The controller sets `error_code = COMM_ERROR` in the status struct.  
- Error is reported on next `GET_STATUS` request.  

---

## 🔄 Example Transactions

### 1. Calibrate Motor
```
Master → ESP32:
[0x10] [0x01]
```

### 2. Set Target = 10000
```
Master → ESP32:
[0x10] [0x02] [0x10] [0x27] [0x00] [0x00]   // little endian 10000
```

### 3. Stop Motor
```
Master → ESP32:
[0x10] [0x03]
```

### 4. Request Status
```
Master → ESP32:
[0x10] [0x05]

ESP32 → Master:
[16-byte MotorStatus struct]
```

---

## 📂 Implementation Notes
- `onReceive()` handles **command parsing**.  
- `onRequest()` handles **status transmission**.  
- The last received command is tracked to decide if a response is needed.  

```cpp
void onReceive(int numBytes) {
  uint8_t cmd = Wire.read();
  switch(cmd) {
    case 0x01: startCalibration(); break;
    case 0x02: setTarget(); break;
    case 0x03: stopMotor(); break;
    case 0x04: deviceReset(); break;
    case 0x05: /* handled in onRequest */ break;
    default:   /* set error_code = COMM_ERROR */ break;
  }
}

void onRequest() {
  if (lastCommand == 0x05) {
    Wire.write((uint8_t*)&status, sizeof(status));
  }
}
```

---

📖 This document defines the **I²C protocol contract** between the ESP32 motor controller and the master device.
