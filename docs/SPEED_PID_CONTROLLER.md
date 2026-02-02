# Speed PID Controller

**Task ID:** 003a-speed-pid-controller  
**Team:** Team B (AXEL)  
**Mission:** Maintain constant velocity

## Overview

Closed-loop PID controller for velocity regulation using IMU-based feedback. Compensates for battery voltage drop and friction variations without requiring reflashing.

## Architecture

```
IMU (BNO055) → Global Variables → PID Controller → CSpeedingMotor → ESC → Motor
                  ↓
            Velocity Feedback (mm/s × 1000)
```

## Serial Commands

All commands use BFMC format: `#command:value;;\r\n`

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| `pidGains` | `Kp;Ki;Kd` | `#pidGains:300;30;5;;\r\n` | Set gains (×100 scale) |
| `pidTarget` | `speed_mm_s` | `#pidTarget:200;;\r\n` | Target speed in mm/s |
| `pidEnable` | `0\|1` | `#pidEnable:1;;\r\n` | Enable/disable PID |
| `pidStatus` | `0` | `#pidStatus:0;;\r\n` | Query current state |

### Response Format

```
@pidStatus:enabled;target;currentVel;error;Kp;Ki;Kd;;\r\n
```

## Default PID Gains

Tuned for Reely TC-04 chassis + Quickrun Fusion SE motor:

| Parameter | Value | Scaled | Purpose |
|-----------|-------|--------|---------|
| Kp | 3.0 | 300 | Speed error correction |
| Ki | 0.3 | 30 | Steady-state error (battery drop) |
| Kd | 0.05 | 5 | Oscillation damping |

## Usage Example

```bash
# Connect via PuTTY (115200 baud)
# 1. Set KL30 mode
#kl:30;;\r\n

# 2. Enable IMU
#imu:1;;\r\n

# 3. Configure PID (optional - defaults are good)
#pidGains:300;30;5;;\r\n

# 4. Set target speed
#pidTarget:200;;\r\n

# 5. Enable PID
#pidEnable:1;;\r\n

# 6. Monitor status
#pidStatus:0;;\r\n
```

## BFMC Competition Notes

- **Highway minimum:** 400 mm/s  
- **City minimum:** 200 mm/s  
- **Max speed:** ±500 mm/s  
- **Anti-windup** prevents integral saturation during stops

## Files Modified

| File | Change |
|------|--------|
| `include/periodics/speedpidcontroller.hpp` | **NEW** - PID controller header |
| `source/periodics/speedpidcontroller.cpp` | **NEW** - PID implementation |
| `include/brain/globalsv.hpp` | Added velocity globals |
| `source/brain/globalsv.cpp` | Added velocity globals |
| `source/periodics/imu.cpp` | Export velocity to globals |
| `source/main.cpp` | Integrate PID + serial commands |

## Derivative Low-Pass Filter (Optional)

The controller includes an optional EMA filter to reduce noise amplification on the derivative term.

**Location:** `source/periodics/speedpidcontroller.cpp` constructor

```cpp
// To DISABLE filter (use raw derivative):
m_useDerivativeFilter = false;

// To ENABLE filter (default):
m_useDerivativeFilter = true;

// Adjust filter strength (0-100):
// Higher = more smoothing, slower response
// Lower  = less smoothing, faster response  
m_derivativeFilterAlpha = 70;  // Default: 70% old + 30% new
```

## Troubleshooting

**PID not responding:**
- Check `#kl:30;;` is set
- Enable IMU with `#imu:1;;`
- Enable PID with `#pidEnable:1;;`

**Oscillation:**
- Reduce Kp: `#pidGains:200;30;5;;`
- Increase Kd: `#pidGains:300;30;10;;`

**Slow response to battery drop:**
- Increase Ki: `#pidGains:300;50;5;;`
