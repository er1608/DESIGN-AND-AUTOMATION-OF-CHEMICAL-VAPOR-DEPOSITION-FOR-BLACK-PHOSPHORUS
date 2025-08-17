
# Design and automation of chemical vapor deposition for black phosphorus

## 1. Introduction

This project focuses on designing and automating the Chemical Vapor Deposition (CVD) process to synthesize Black Phosphorus. It integrates hardware control STM32 with a user-friendly graphical interface for monitoring and management.

## 2. System Requirements

Hardware: STM32 development board, thermocouples, PID unit, heating element, etc.

Software:

    Python ≥ 3.x

    GUI library (PyQt or Tkinter)

    STM32CubeMX & toolchain for STM32

    matplotlib or similar for plotting

## 3. Installation & Getting Started

1) Clone the repository and checkout the Khoa branch:

    git clone <repo_url>
    cd DESIGN-AND-AUTOMATION-OF-CHEMICAL-VAPOR-DEPOSITION-FOR-BLACK-PHOSPHORUS
    git checkout Khoa

2) Set up Python environment and dependencies
3) Flash the firmware into STM32 via STM32CubeMX/IDE.

4) Run the GUI:

    python CONTROL_GUI.py

## 4. GUI usage

Provides controls for switching heating on/off, tuning PID parameters, and monitoring temperature in real time.

Displays graphical plots of the CVD process.

Changed time parameters for 5 stage:

    T1: temperature change time of stage 1

    T2: temperature change time of stage 2

    T3: temperature change time of stage 3

    T4: temperature change time of stage 4
    
    T5: temperature change time of stage 5

Changed temperature parameters for 5 stage:

    S1: The amount of temperature increase after each time period of stage 1

    S2: Desired temperature after completion of stage 1 and maintained in stage 2

    S3: The amount of temperature decrease after each time period of stage 3 and 5

    S4: Desired temperature after completion of stage 3 and maintained in stage 4

## 5. Control Programming & Drivers

Drivers/ includes ADC, GPIO, SPI, UART code.

Core handles PID algorithms and process control logic.

GUI enables intuitive visualization and user interaction.
