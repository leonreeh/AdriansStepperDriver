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
- **Motor Control Core**
  - Timer-driven step pulse generation
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
- `0x02` → `SET_TARGET_ABS [position]`
- `0x03` → `MOVE_REL [delta]`
- `0x04` → `STOP`
- `0x05` → `RESET`
- `0x06` → `GET_STATUS`

**Status Structure (I²C readback):**
```c
struct MotorStatus {
    int32_t actual_position;     // encoder count
    int32_t target_position;     // commanded position
    int32_t calibration_offset;  // zero reference
    uint16_t motor_state;        // idle, moving, calibrating, error
    uint16_t error_code;         // 0 = ok, 1 = stall, 2 = out-of-range
};
```

---

## 🛠️ Development Roadmap

### Phase 1 – Core Setup
- [ ] Timer-based step pulse generation
- [ ] Encoder quadrature decoding
- [ ] Basic open-loop stepper test

### Phase 2 – Motion Profiles
- [ ] Implement trapezoidal acceleration/deceleration
- [ ] Track internal expected position counter

### Phase 3 – Encoder Integration
- [ ] Periodic correction loop
- [ ] Error handling for persistent mismatch

### Phase 4 – Calibration
- [ ] Stall-detection-based homing sequence
- [ ] Save/load calibration offset from flash

### Phase 5 – I²C Communication
- [ ] Implement command parser
- [ ] Implement status reporting

### Phase 6 – Safety & Reliability
- [ ] Watchdog timer integration
- [ ] Emergency stop
- [ ] Error codes (stall, out-of-range, comm failure)

### Phase 7 – Integration Hell
- [ ] Master device integration
- [ ] Calibration validation

---

## 📂 Repository Structure (planned)
```
/src
  main.c
  motor_control.c
  motor_control.h
  encoder.c
  encoder.h
  i2c_interface.c
  i2c_interface.h
  calibration.c
  calibration.h
/docs
  architecture.md
  command_protocol.md
README.md
```

---

## 🚀 Future Improvements
- diagnostics (via UART-to-USB bridge)
- Support for multiple motor channels
- Advanced motion planning (S-curve profiles)
- Extended error handling & self-diagnostics
