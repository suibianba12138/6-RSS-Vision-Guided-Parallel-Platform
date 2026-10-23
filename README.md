<div align="center">

# 6-RSS Vision-Guided Parallel Platform

### Engineering Motion from Vision

**An experimental six-actuator parallel-platform project connecting computer vision, inverse kinematics, a Python desktop controller, and ESP32 servo control.**

[**🌐 Interactive Project Website**](https://suibianba12138.github.io/6-RSS-Vision-Guided-Parallel-Platform/) · [**System Architecture**](docs/ARCHITECTURE.md) · [**Code Guide**](docs/CODE_GUIDE.md) · [**Source**](调用反解DLL库/)

![Python](https://img.shields.io/badge/Python-Desktop%20Control-3776AB?logo=python&logoColor=white)
![Embedded](https://img.shields.io/badge/Embedded-ESP32-087C9E)
![Vision](https://img.shields.io/badge/Vision-RGB--D%20%2B%20OpenCV-0B7285)
![Status](https://img.shields.io/badge/Status-Experimental%20Prototype-E39717)

</div>

---

<div align="center">

![6-RSS architecture](docs/assets/system-overview.svg)

*An overview of the components present in the source code. The two perception modes are experimental variants, not a single validated unified pipeline.*

</div>

## 🌐 Interactive Project Showcase

[**Explore the 6-RSS project website →**](https://suibianba12138.github.io/6-RSS-Vision-Guided-Parallel-Platform/)

The website introduces the platform with a responsive engineering illustration, an interactive explanation of the **laser-target** and **black-rectangle** vision routes, the control chain and links to the corresponding original implementations. The schematic and interactive walkthrough are **illustrations of source-code modules, not live camera feeds, simulations, or recorded experiments**. The website is maintained in the [`gh-pages` branch](https://github.com/suibianba12138/6-RSS-Vision-Guided-Parallel-Platform/tree/gh-pages).

## 🎬 Demonstration

**Hardware demo video is not included yet.** This section is reserved for a future recording of the physical platform; no generated animation or synthetic video is presented as a real experiment.

For now, the source code documents the PC-to-ESP32 actuation path and two visual-perception/controller experiments.

## ✨ Project Highlights

- **RGB-D perception:** RealSense depth and color capture, with OpenCV-based detection experiments for a red/white laser target and a black rectangular target.
- **Upper-computer control:** Tkinter desktop interfaces for configuring communication, viewing measurements, issuing commands, and running experimental control routines.
- **Kinematic mapping:** A Windows-style compiled inverse-kinematics library exposes `InK6RSS`, accepting a six-component platform pose and returning six actuator angles.
- **Microcontroller interface:** Serial messages carry six angles to an ESP32, whose firmware maps them to six servo outputs via `ESP32Servo`.
- **Trajectory prototype:** One controller variant constructs a square-path sequence from visually estimated rectangle dimensions.

These are **source-level capabilities**, not claims of a calibrated, robust or quantitatively validated closed-loop servo system.

## 🧩 How It Works

```text
RealSense RGB + depth
    |
    +-- Laser-target detection (vision2.py)
    |      \-- Target pixel offset and depth
    |              \-- Experimental proportional pose control (shangweiji11.py)
    |
    +-- Black-rectangle detection (visonblack.py)
           \-- Rectangle dimensions and location
                   \-- Square-path prototype (shangweijiblack4.py)

Pose command (translation + yaw/pitch/roll)
    --> InK6RSS inverse-kinematics DLL
    --> 6 actuator-angle commands
    --> Python serial port
    --> ESP32 firmware (sketch_sep14a.ino)
    --> 6 servo channels
```

Read [**System Architecture**](docs/ARCHITECTURE.md) for the code-level relationships, frame/units caveats and limitations.

## 📂 Repository Guide

| Path | Purpose |
|---|---|
| [`sketch_sep14a/sketch_sep14a.ino`](sketch_sep14a/sketch_sep14a.ino) | ESP32 serial receiver and six-servo output mapping |
| [`调用反解DLL库/shangweiji11.py`](调用反解DLL库/shangweiji11.py) | Tkinter controller experiment using laser-target detection |
| [`调用反解DLL库/vision2.py`](调用反解DLL库/vision2.py) | Adjustable red/white laser feature detection |
| [`调用反解DLL库/shangweijiblack4.py`](调用反解DLL库/shangweijiblack4.py) | Black-rectangle-guided square trajectory experiment |
| [`调用反解DLL库/visonblack.py`](调用反解DLL库/visonblack.py) | RealSense and black-rectangle detection utilities |
| [`调用反解DLL库/InK6RSSV1.h`](调用反解DLL库/InK6RSSV1.h) | Kinematic definitions and inverse-solver interface |
| [`调用反解DLL库/RSS6RBT_InverseDLL.dll`](调用反解DLL库/RSS6RBT_InverseDLL.dll) | Existing compiled native solver; provenance/licensing still to be checked |

Additional `shangweiji*.py` scripts are retained as **historical experiments**, not independently verified release versions. See the [**Code Guide**](docs/CODE_GUIDE.md).

## 🚀 Running the Prototype

**Hardware-dependent; not a one-command reproducible release.** The repository has no locked dependency file, integration tests or verified setup procedure as of this documentation update.

Source imports indicate the following environment components: Python with `numpy`, `opencv-python`, `pyserial`, `Pillow`, `pyrealsense2`; Tkinter support; a compatible Intel RealSense RGB-D camera; an ESP32 board with `ESP32Servo`; six servos and an appropriate power supply; and an OS/runtime compatible with the provided native inverse solver.

The controller currently loads `./RSS6RBT_InverseDLL.dll` **relative to the process working directory**. Start scripts from the solver directory after reviewing the code and verifying compatibility with your hardware. Treat that DLL as an **unverified third-party/native binary** until its origin and usage terms are confirmed. Do not execute it on a system you cannot trust.

The ESP32 source currently expects serial frames such as:

```text
S,angle1,angle2,angle3,angle4,angle5,angle6,E
```

Default baud rate: **115200**. The firmware additionally transforms alternating indexed angles before issuing servo writes. Review motor directions, travel limits and mechanical interference **before enabling power**; do not use the committed settings as universal servo calibrations.

## ⚠️ Limitations & Validation Status

This repository is an **archived educational/experimental engineering prototype**, not a deployable robot control package.

- The available source shows integration intent and implemented module boundaries; **end-to-end hardware execution was not independently revalidated** for this documentation update.
- Camera-to-platform coordinates, depth-to-displacement conversion, and yaw/pitch command math are experimental and require calibration.
- The source contains multiple historical controller revisions; no single canonical validated configuration or measured control accuracy is currently documented.
- The project has no documented repeatable benchmark for tracking error, latency, stability or trajectory accuracy.
- Servo range checking, safe initialization, and command handling require further review before unattended operation.
- Repository history contains generated and recorded files that may be unnecessary for a public source release; see [**Code Guide**](docs/CODE_GUIDE.md).

## 📜 Attribution & Usage

Authorship and redistribution terms for the included `RSS6RBT_InverseDLL.dll` and its associated interface are **not established in this repository**. Their presence on a public repository does not imply permission to redistribute or relicense them. No project-wide open-source license is asserted by this README.

Source credits and any original third-party documentation should be verified before repackaging or publishing a cleaned release.

---

<div align="center"><sub>6-RSS Parallel Platform · Vision, Kinematics & Embedded Control · Experimental Engineering Showcase</sub></div>
