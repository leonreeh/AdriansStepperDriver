# I²C Interface & Register Map

## 📌 Overview
The ESP32 stepper motor controller communicates with a master device over **I²C** in **slave mode**.  
Commands are issued from the master, and status data can be requested for monitoring or error handling.

### Default Slave Address
```cpp
#define DEFAULT_I2C_ADDRESS 0x10
uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;
```
The I²C address is stored in non-volatile memory and can be customized for multi-device setups.

---

## ⚙️ Command Set

### **0x01 – CALIBRATE**
- **Direction:** Master → Slave  
- **Payload:** none  
- **Action:**  
  - Runs the homing routine using stall detection.  
  - Applies a configured **home offset** to define logical position `0`.  
  - Stores zero offset and sets current position as **absolute reference** in NVS.  
- **Response:** none  

---

### **0x02 – SET_TARGET**
- **Direction:** Master → Slave  
- **Payload:**  
  - 4 bytes (signed 32-bit integer, little endian) representing the **target position in steps** relative to logical zero.  
- **Action:**  
  - Motor moves to requested position using open-loop motion with encoder feedback.  
  - Target is clamped to the **maximum travel range** (0…`MAX_RANGE_STEPS`).  
- **Response:** none  

---

### **0x03 – STOP MOTOR**
- **Direction:** Master → Slave  
- **Payload:** none  
- **Action:**  
  - Immediately stops motion and stores current position as last known absolute position.  
- **Response:** none  

---

### **0x04 – RESET DEVICE**
- **Direction:** Master → Slave  
- **Payload:** none  
- **Action:**  
  - Resets state machine and clears errors.  
  - Restores last known absolute position from NVS memory.  
- **Response:** none  

---

### **0x05 – GET STATUS**
- **Direction:** Master → Slave  
- **Payload:** none  
- **Response:** Returns a **16-byte struct** containing motor status.

```c
struct MotorStatus {
    int32_t actual_position;     // Current logical position in steps
    int32_t target_position;     // Last commanded target in steps
    int32_t calibration_offset;  // Logical offset (0 after homing)
    uint16_t motor_state;        // 0=idle, 1=moving, 2=calibrating, 3=error, 4=stopped
    uint16_t error_code;         // 0=ok, 1=stall, 2=out-of-range, 3=calibration error, 6=communication error
};
```

---

## 🛠️ Error Handling
| Error Code | Meaning                         | Recovery                                   |
|------------|---------------------------------|---------------------------------------------|
| `0`        | OK                              | —                                           |
| `1`        | Stall detected                  | Check mechanical load, reset device         |
| `2`        | Target out of allowed range     | Send target within `0…MAX_RANGE_STEPS`      |
| `3`        | Calibration error               | Retry calibration, inspect end-stop setup   |
| `6`        | Communication error             | Check I²C bus integrity                     |

---

## 🔄 Example Transactions

### **1. Calibrate Motor**
```
Master → ESP32:
[0x10] [0x01]
```

---

### **2. Set Target = 10000**
```
Master → ESP32:
[0x10] [0x02] [0x10] [0x27] [0x00] [0x00]   // little endian 10000
```

---

### **3. Stop Motor**
```
Master → ESP32:
[0x10] [0x03]
```

---

### **4. Request Status**
```
Master → ESP32:
[0x10] [0x05]

ESP32 → Master:
[16-byte MotorStatus struct]
```

---

## 📂 Implementation Notes
- `onReceive()` handles **command parsing and validation**.  
- `onRequest()` serves **status data** when the last command was `GET_STATUS`.  
- The firmware **persists last known absolute position** across reboots for consistent motion planning.

```cpp
void onReceive(int numBytes) {
  uint8_t cmd = Wire.read();
  switch(cmd) {
    case 0x01: setState(SM_CALIBRATING); break;
    case 0x02: setTarget(target); break;
    case 0x03: stopMotor(); break;
    case 0x04: deviceReset(); break;
    case 0x05: /* handled in onRequest */ break;
    default:   setError(ERR_COM); break;
  }
}

void onRequest() {
  if (lastCommand == 0x05) {
    Wire.write((uint8_t*)&status, sizeof(status));
  }
}
```

---

This defines the complete **I²C communication contract** for motion control, homing, error reporting, and state persistence.
