# RocketPower

<p align="center">
<img src="Media/CAD/FullRocket.png" alt="Full rocket CAD render" height="600">
<img src="Media/CAD/WithoutBodyCoverandNoseCone.png" alt="Cutaway CAD render, body cover and nose cone removed" height="600">
</p>

A model rocket built **from the ground up**: custom flight computer, custom PCB, active fin stabilization, and a full hardware-in-the-loop test rig.

**Status:** in bench and hardware-in-the-loop validation ahead of first flight.

---

## 🔧 Putting it together

<table><tr>
<td width="62%"><img src="Media/web/avionics_bay.gif" alt="Avionics bay assembly" width="100%"></td>
<td width="38%"><img src="Media/CAD/Avionics%20Bay.png" alt="Avionics bay CAD render" width="100%"></td>
</tr></table>

---

## 🚀 What it does

- **Active attitude control.** An ARM Cortex-M7 flight computer runs a PD controller on quaternion-derived tilt plus gyro rate, driving **4 independent servo fins** in real time.
- **Redundant recovery.** Apogee detection runs on multiple independent signals, each able to trigger deployment on its own.
- **Hardware-in-the-loop testing.** A custom 6DOF Simulink model drives the real flight computer over serial, so the control loop gets validated **before it ever has to fly**.
- **Custom PCB.** IMU, barometer, high-g accelerometer, GPS, power monitoring, pyro firing and continuity, and LoRa telemetry, all on one board.
- **Ground station.** A PyQt6 desktop app for live telemetry, 3D orientation, and command/control.

---

## 🧠 Architecture

Every sensor feeds one MCU; that MCU runs the control law and drives every output.

```mermaid
flowchart TD
    classDef sensor fill:#4A90D9,stroke:#2C5F8A,color:#fff
    classDef compute fill:#F5A623,stroke:#B8780F,color:#000
    classDef output fill:#7ED321,stroke:#4C8A14,color:#000
    classDef ext fill:#9B59B6,stroke:#6C3483,color:#fff

    IMU[BNO085 IMU]:::sensor
    BARO[BMP390 Baro]:::sensor
    ACC[ADXL375 Accel]:::sensor
    GPS[ZOEM8 GPS]:::sensor
    PWR[INA260 Power]:::sensor

    subgraph MCU["ARM Cortex-M7 MCU (NXP i.MX RT1062)"]
        FSM[Flight State Machine]:::compute
        CTRL[PD Attitude Controller]:::compute
        PYRO[Pyro + Backup Deploy]:::compute
    end

    FINS[4x Servo Fins]:::output
    CHUTE[Recovery Parachute]:::output

    GS[GroundStation]:::ext
    SIM[Simulink 6DOF]:::ext

    IMU --> FSM
    BARO --> FSM
    ACC --> FSM
    GPS --> FSM
    PWR --> FSM
    FSM --> CTRL
    FSM --> PYRO
    CTRL --> FINS
    PYRO --> CHUTE
    FSM <-->|"LoRa, E22-900T22D"| GS
    FSM <-.->|"Serial, HITL only"| SIM

    style MCU fill:#FEF3E0,stroke:#F5A623,stroke-width:2px
```

---

## 🛠️ Hardware

Every subsystem on one custom board: IMU, barometer, high-g accelerometer, GPS, power monitoring, pyro firing/continuity, and telemetry.

<p align="center">
<img src="Media/LinkedIn_Post1/PCBLAYOUT.png" alt="PCB layout" height="320">
<img src="Media/web/pcb_render.gif" alt="PCB 3D render" height="320">
</p>

---

## 🧪 Simulation and testing

Performance is modeled before anything is built, and the same model closes the loop with real hardware once it exists.

<p align="center">
<img src="Media/Simulation/Simulink.png" alt="Simulink model view" height="320">
<img src="Media/web/flight_path_animation.gif" alt="Flight path animation" height="320">
</p>

A full powered ascent, coast, and parachute deployment, run entirely in hardware-in-the-loop: **real flight computer, simulated flight.**

![Hardware-in-the-loop test](Media/web/hero_highlight.gif)

---

<p align="center"><img src="Media/Rocket%20Launch/RocketLaunch.png" alt="Reference launch photo" width="40%"></p>
<p align="center"><sub>Where this is headed.</sub></p>
