# RAVEN - Embedded Control ("The Spinal Cord")

![Raven Embedded](https://img.shields.io/badge/Component-Embedded-orange) ![Status](https://img.shields.io/badge/Status-Active-success)

The **Embedded Control** firmware runs on the **STM32 Nucleo** board. It provides low-level hardware abstraction, real-time motor control, and safety features.

## 🌍 Global System Architecture

```mermaid
graph TD
    %% Styling
    classDef infra fill:#2d3748,stroke:#4a5568,color:#fff
    classDef web fill:#2b6cb0,stroke:#63b3ed,color:#fff
    classDef computer fill:#1A4024,stroke:#3D9955,color:#fff
    classDef brain fill:#6b46c1,stroke:#b794f4,color:#fff
    classDef perception fill:#805ad5,stroke:#b794f4,color:#fff
    classDef embedded fill:#744210,stroke:#d69e2e,color:#fff
    classDef sim fill:#2c7a7b,stroke:#4fd1c5,color:#fff
    classDef docs fill:#4a5568,stroke:#a0aec0,color:#fff

    subgraph "Infrastructure & Tools"
        CLI[raven-infrastructure CLI]:::infra
        DOCS[raven-documentation Sphinx]:::docs
    end

    subgraph "Presentation & User Interfaces"
        FUTURA[axel-launch-futura<br>React/Vite Official Web]:::web
        DASH[raven-computer<br>Remote Dashboard / Telemetry]:::computer
    end

    subgraph "High-Level Control (Raspberry Pi)"
        BRAIN[raven-brain-stack<br>Python Multiprocessing]:::brain
        
        subgraph "Perception Pipeline"
            CAMERA[Camera / Frame Receiver]:::perception
            LANEDET[Lane Detection<br>OpenCV IPM & Polyfit]:::perception
            SIGNDET[Sign Detection<br>YOLOv8 + Filters]:::perception
            LOCALIZATION[Localization<br>Odometry & Map Graph]:::perception
            CAMERA --> LANEDET
            CAMERA --> SIGNDET
        end
        
        PLANNER[Planner / FSM<br>Decision Logic]:::brain
        SERIAL[Serial Communication<br>Asynchronous UART]:::brain

        LANEDET -->|Lateral Offset| PLANNER
        SIGNDET -->|Sign Labels| PLANNER
        LOCALIZATION <-->|Pose & Drift Correct| PLANNER
        PLANNER -->|Speed & Steer Commands| SERIAL
    end

    subgraph "Low-Level Control (STM32/RP2040)"
        EMBEDDED[raven-embedded-control C++]:::embedded
        PID[Speed PID & Steering MPC]:::embedded
        SAFETY[Dead Man Switch]:::embedded

        EMBEDDED --- PID
        EMBEDDED --- SAFETY
    end

    subgraph "Simulation"
        SIM[raven-sim Gazebo ROS1]:::sim
    end

    %% Connections
    CLI -.->|"Deploys/Manages"| BRAIN
    CLI -.->|"Flashes"| EMBEDDED
    DASH <-->|"Socket.IO / Telemetry"| BRAIN
    SERIAL <-->|"USB Serial (#cmds / @telemetry)"| EMBEDDED
    SIM -.->|"Provides Camera / ROS / Telemetry"| BRAIN
    SIM <-->|"Simulated Server"| DASH
```

## 📚 Documentation
> **Full Technical Documentation:** [bosch-future-mobility-challenge-documentation.readthedocs-hosted.com](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com)

---

## 🚀 Key Features

| Task ID | Feature Name | Description |
| :--- | :--- | :--- |
| **[003a]** | **Speed PID Controller** | Closed-loop velocity control using IMU feedback and dynamic gain tuning. |
| **[003b]** | **Steering & MPC** | Servo control with anti-jitter and Model Predictive Control command support. |
| **[004a]** | **Message Lexer** | Efficient parsing of incoming serial packets (e.g., `#SPEED:15.5;;`). |
| **[004b]** | **Command Parser** | Routes parsed commands to appropriate subsystems (Motors, Sensors). |
| **[004c]** | **Dead Man's Switch** | Safety watchdog that stops the car if the Brain disconnects (>500ms). |

## 🛠️ Usage

### Flashing
Use Mbed Studio or CLI to compile and flash:
```bash
mbed compile -t GCC_ARM -m NUCLEO_F401RE --flash
```


### Serial Commands
Connect via USB (Baud: 115200) to send manual commands:
- `#speed:20.0;;` (Set speed to 20 cm/s)
- `#steer:15.0;;` (Set steer angle to 15 deg)
- `#brake:1;;` (Emergency Stop)

---

## ⚡ Arduino Nano RP2040 Connect (New)

We now support the Arduino Nano RP2040 Connect as an alternative to the Nucleo.

### Pinout (RP2040 Connect)
- **Wheel Encoders**:
  - `CLK`: Pin 2 (Hardware Interrupt)
  - `DT`: Pin 3
- **Speed Motor (L298N)**:
  - `IN1`: Pin 7
  - `IN2`: Pin 8
  - `EN`: Pin 9 (PWM)
- **Steering Servo**:
  - `Signal`: Pin 11
- **IMU**: Built-in LSM6DSOX

### Flashing (Arduino-CLI)
Use the `raven` CLI to flash the Arduino firmware:
```bash
raven flash --arch arduino
```