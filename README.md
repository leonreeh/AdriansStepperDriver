# AdriansNippleTwister

## 📌 Overview
This project implements firmware for an **ESP32-based stepper motor controller** with **encoder feedback**.  
The controller drives the motor in open-loop mode while applying **periodic encoder-based corrections** for better accuracy and reliability.  

It communicates with a **master device via I²C**, which sends commands for calibration, motion, or status queries.  
The firmware also supports **persistent position storage**, so after a power cycle, the controller resumes knowing its last absolute position.

---

## ✨ Key Features
- **Absolute motion control**  
  Targets are always relative to a logical home position.
- **Persistent position memory**  
  Remembers its last absolute position after power loss.
- **Configurable homing offset**  
  Logical zero can be offset from the mechanical stop.
- **Soft travel limits**  
  Prevents moves outside the defined travel range.
- **Stall detection**  
  Detects missed steps during motion and calibration.
- **Idle drift correction**  
  Keeps position consistent even if moved by external forces.
- **I²C slave interface**  
  Supports real-time control and status monitoring.

---

## ⚙️ System Architecture

### Internal Modules
- **Motion Controller**
  - Trapezoidal acceleration/deceleration profiles via AccelStepper.
  - Absolute position reference for all motion commands.
- **Encoder Feedback**
  - Quadrature decoding via ESP32 timers.
  - Used for drift correction and stall detection.
- **Calibration (Homing)**
  - Moves until a mechanical stop is detected.
  - Applies a configurable offset (`HOME_OFFSET_STEPS`) to define logical zero.
- **Persistence Layer**
  - Stores last known position and calibration info in NVS (Preferences).
- **I²C Interface**
  - Receives commands from the master.
  - Sends back structured status data.

---

## 🔌 I²C Command Set

| Command | Code | Payload | Description |
|----------|------|---------|-------------|
| CALIBRATE | `0x01` | None | Runs homing and zeroing routine |
| SET_TARGET | `0x02` | 4 bytes (int32 target) | Moves to absolute position from logical zero |
| STOP | `0x03` | None | Immediately stops the motor |
| RESET | `0x04` | None | Resets internal state and preferences |
| GET_STATUS | `0x05` | None | Returns the status struct |

---

## 📊 Status Structure

```c
struct MotorStatus {
    int32_t actual_position;    // Logical position (steps from zero)
    int32_t target_position;    // Commanded position
    int32_t calibration_offset; // Offset from mechanical stop
    uint16_t motor_state;       // 0=idle,1=moving,2=calibrating,3=error,4=stop
    uint16_t error_code;        // 0=ok,1=stall,2=out-of-range,3=cal error,6=comm error
};
```

---

## 🛠️ Development Roadmap

### Completed
- ✅ Motion control with trapezoidal profiles
- ✅ Encoder integration and drift correction
- ✅ Homing with configurable offset
- ✅ Persistent position storage
- ✅ I²C command parser and status reporting
- ✅ Stall detection logic
- ✅ Soft travel limits

### Next Steps
- [ ] Diagnostic tools via UART/USB
- [ ] Extended error handling & self-tests
- [ ] Integration Hell

---

## 📂 Repository Structure

```
/src
  main.ino
/docs
  I2C_Interface.md
  README.md
```

---

