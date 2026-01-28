# BFMC - Embedded platform project

The project contains all the software present on the Nucleo board, together with the documentation on how to create new components and what are the features of the given one. Some of the feature are:
- Communication protocol between RPi and Nucleo,
- Motors control,
- IMU readings
- Notifications from Power Board
- Architecture prone to features addition

## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/embeddedplatform.html) 

---

## 🦅 AXEL Team Modifications

This fork contains modifications by **Team AXEL** for the 2025-26 BFMC competition. Below are the changes from the [original Bosch repository](https://github.com/ECC-BFMC/Embedded_Platform).

### [003a] Speed PID Controller

**Branch:** `003a-speed-pid-controller`  
**Mission:** Maintain constant velocity using closed-loop control.

#### New Features
- **Closed-loop velocity control** using IMU-based feedback
- **Serial-configurable PID gains** — tune without reflashing!
- **Anti-windup protection** for stable response
- **Feedforward compensation** for fast setpoint tracking

#### New Serial Commands
| Command | Format | Description |
|---------|--------|-------------|
| `pidGains` | `Kp;Ki;Kd` (×100) | Set PID gains at runtime |
| `pidTarget` | `speed_mm_s` | Set target speed |
| `pidEnable` | `0\|1` | Enable/disable PID |
| `pidStatus` | `0` | Query controller state |

#### Files Changed
| File | Modification |
|------|--------------|
| `include/periodics/speedpidcontroller.hpp` | **NEW** |
| `source/periodics/speedpidcontroller.cpp` | **NEW** |
| `include/brain/globalsv.hpp` | + velocity globals |
| `source/brain/globalsv.cpp` | + velocity globals |
| `source/periodics/imu.cpp` | + velocity export |
| `source/main.cpp` | + PID integration |
| `docs/SPEED_PID_CONTROLLER.md` | **NEW** - Full documentation |

📖 **Full documentation:** [docs/SPEED_PID_CONTROLLER.md](docs/SPEED_PID_CONTROLLER.md)

---

### Future Tasks
- [ ] 003b - Steering Calibration
- [ ] 003c - Lane Keeping
- [ ] 004 - Traffic Sign Detection 