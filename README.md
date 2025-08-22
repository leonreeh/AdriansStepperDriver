# AdriansNippleTwister

## 📌 Overview
This project implements firmware for an **ESP32-based stepper motor controller** with **encoder feedback**.  
The controller runs open-loop motion profiles with **periodic encoder correction**, ensuring accurate positioning while minimizing CPU overhead - und weil ich dir keine PID loop programmiere hurensohn.

It communicates with a **master device via I²C**, which sends motion commands and retrieves status information.

### ✨ Key Features
- Stepper motor drive with **encoder feedback**
- Open-loop motion with **periodic corrections**
- **Calibration via stall detection** (endstop finding)
- Absolute and relative motion commands
- I²C slave interface for control and monitoring
- Status reporting (actual/target positions, calibration offset, state, errors)
- Safety features: watchdog timer, emergency stop, stall detection

---

## ⚙️ System Architecture

### Internal Modules
- **Motion Profiles**
  - Basic open-loop stepper motion
  - Trapezoidal acceleration/deceleration motion profiles
- **Encoder Interface**
  - Quadrature decoding via ESP32 timers
  - Real-time position feedback
- **Correction Loop**
  - Periodically compares commanded vs encoder position
  - Applies corrections or flags stall errors
- **Calibration**
  - Runs homing sequence using stall detection
  - Stores calibration offset in Flash/EEPROM
- **I²C Slave Communication**
  - Command parser for CALIBRATE, MOVE, STOP, RESET
  - Status structure reporting

### External Interface (I²C Master ↔ ESP32)
**Supported Commands:**
- `0x01` → `CALIBRATE`
- `0x02` → `SET_TARGET [position]`
- `0x03` → `STOP`
- `0x04` → `RESET`
- `0x05` → `GET_STATUS`

**Status Structure (I²C readback):**
```c
struct MotorStatus {
    int32_t actual_position;     // encoder count
    int32_t target_position;     // commanded position
    int32_t calibration_offset;  // zero reference
    uint16_t motor_state;        // 0=idle, 1=moving, 2=calibrating, 3= error, 4= stop
    uint16_t error_code;         // 0=ok, 1=stall, 2=out-of-range, 3= Calibration error, 4= tbd, 5= tbd, 6=comm error
};
```

---

## 🛠️ Development Roadmap

### Phase 1 – Motion Profiles
- [ ] Basic open-loop stepper motion
- [ ] Implement trapezoidal acceleration/deceleration
- [ ] Track internal expected position counter

### Phase 2 – Encoder Integration
- [ ] Encoder quadrature decoding
- [ ] Periodic correction loop
- [ ] Error handling for persistent mismatch

### Phase 3 – Calibration
- [ ] Stall-detection-based homing sequence
- [ ] Save/load calibration offset from flash

### Phase 4 – I²C Communication
- [x] Implement command parser
- [x] Implement status reporting

### Phase 5 – Safety & Reliability
- [ ] Emergency stop
- [ ] Stall detection
- [x] Error codes

### Phase 6 – Integration Hell
- [ ] Master device integration
- [ ] Calibration validation

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

## 🚀 Future Improvements
- diagnostics (via UART-to-USB bridge)
- Support for multiple motor channels
- Advanced motion planning (S-curve profiles)
- Extended error handling & self-diagnostics
