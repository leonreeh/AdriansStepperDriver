# Configuration & Tuning Guide

This document explains how to configure and tune the ESP32 Stepper Motor Controller.  
All constants are located at the **top of `main.ino`** and are grouped by feature.

---

## 1. Stall Detection Tuning
Stall detection helps detect when the motor stops moving unexpectedly, either due to a jam, reaching a hard stop, or losing torque.

| Parameter | Default | Description |
|------------|---------|-------------|
| `STALL_WINDOW_MS` | `25` ms | Time window for detecting movement. Lower values = faster reaction but more noise. |
| `STALL_MIN_EXPECTED_STEPS` | `2` steps | Ignore tiny moves in a window. |
| `STALL_MIN_RATIO` | `0.30` | Encoder vs. step ratio threshold. Increase to make stall detection stricter. |
| `STALL_CONSECUTIVE_WINDOWS` | `6` windows (~150 ms) | Number of bad readings before declaring a stall. |
| `STALL_IGNORE_SPEED_STEPS_S` | `20.0` steps/s | Ignore stall detection below this speed (low-speed noise filter). |
| `ENC_COUNTS_PER_STEPPER_STEP` | `1.0` | Encoder counts per step. Adjust to match your encoder resolution and gear ratio. |

---

## 2. Calibration / Homing Tuning
Homing uses stall detection to find the mechanical zero (hard stop), then backs off to a logical zero point.

| Parameter | Default | Description |
|------------|---------|-------------|
| `HOMING_TOWARD_MIN` | `true` | `true` = home towards negative direction, `false` = positive direction. |
| `HOMING_FAST_MAX_SPEED` | `1200` steps/s | Speed for fast approach phase. |
| `HOMING_SLOW_MAX_SPEED` | `300` steps/s | Speed for slow precise seek phase. |
| `HOMING_ACCEL` | `400` steps/s² | Acceleration during homing. |
| `HOMING_BACKOFF_STEPS` | `400` steps | Steps to back off after first stall. |
| `HOMING_STALL_SETTLE_MS` | `150` ms | Pause after detecting stall before continuing. |
| `HOMING_MAX_TRAVEL_FAST` | `20000` steps | Safety cap for fast seek travel. |
| `HOMING_MAX_TRAVEL_SLOW` | `4000` steps | Safety cap for slow seek travel. |
| `HOME_OFFSET_STEPS` | `150` steps | Logical zero offset from the hard stop. |

---

## 3. Motion Profile Settings

| Parameter | Default | Description |
|------------|---------|-------------|
| `MaxSpeed` | `1000.0` steps/s | Default maximum movement speed. |
| `MaxAcceleration` | `200.0` steps/s² | Default acceleration. |
| `MAX_RANGE_STEPS` | `12000` steps | Maximum allowed travel range from logical zero. |

---

## 4. Encoder Configuration

| Parameter | Default | Description |
|------------|---------|-------------|
| `ENC_A` | `34` | Encoder channel A pin. |
| `ENC_B` | `35` | Encoder channel B pin. |
| `encoder_origin_offset` | dynamic | Logical offset to sync encoder counts after boot or homing. |

---

## 5. I²C Configuration

| Parameter | Default | Description |
|------------|---------|-------------|
| `DEFAULT_I2C_ADDRESS` | `0x10` | Default address for I²C communication. |
| `SECOND_I2C_ADDRESS` | `0x11` | Optional alternate address. |
| `THIRD_I2C_ADDRESS` | `0x20` | Optional alternate address. |
| `FORTH_I2C_ADDRESS` | `0x21` | Optional alternate address. |

---

## 6. Persistence Behavior

The controller uses ESP32 **Preferences (NVS)** to store:
- `last_pos` → last logical position on power-off  
- `cal_offset` → calibration offset (usually `0` with logical zero at `HOME_OFFSET_STEPS`)  
- `i2c_addr` → configurable I²C address  

Data is automatically updated during:
- Position changes (Idle or Stop states)
- Calibration completion
- Device reset

---

## 7. Practical Tuning Tips
- **Stall sensitivity**:  
  - Too many false positives? Decrease `STALL_MIN_RATIO` or increase `STALL_CONSECUTIVE_WINDOWS`.  
  - Missed stalls? Increase `STALL_MIN_RATIO` or decrease `STALL_CONSECUTIVE_WINDOWS`.
- **Homing issues**:  
  - Adjust `HOMING_FAST_MAX_SPEED` and `HOMING_SLOW_MAX_SPEED` for your motor strength.  
  - Increase `HOMING_BACKOFF_STEPS` if it doesn’t clear the stop during backoff.
- **Travel limits**:  
  - Adjust `MAX_RANGE_STEPS` to match your mechanism’s physical range.
- **Encoder calibration**:  
  - Set `ENC_COUNTS_PER_STEPPER_STEP` accurately for correct stall detection and diagnostics.

---

## 8. Example Workflow
1. Perform **calibration** with command `0x01` (homing).  
2. Send a **target position** with command `0x02`.  
3. Check travel limits and stall detection to ensure reliable motion.  
4. Tune stall parameters if needed based on observed performance.

---

This guide should be referenced whenever hardware or motion tuning is required.
