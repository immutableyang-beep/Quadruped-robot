# Quadruped Robot (PIC24)

This project implements a quadruped walking robot based on a **PIC24FJ64GA002** microcontroller.
The robot uses an external PWM driver to control multiple servos and performs basic locomotion through timer-driven motion logic.

This project was developed as a hands-on embedded systems project, with an emphasis on **low-level hardware control**, **timer interrupts**, and **state-based motion design**.

---

## Features

* Quadruped walking using coordinated leg movement
* **8-servo control** via external PWM driver
* Timer-interrupt-based motion control
* Adjustable walking speed
* Button-controlled motion modes
* Direct register-level programming on PIC24

---

## Hardware Used

* **Microcontroller:** PIC24FJ64GA002
* **Servo Driver:** PCA9685 (I2C)
* **Servos:** 8 total (2 per leg)
* Push buttons for user input
* External power supply for servos

---

## Software Structure

src/
├── main.c                  # Main control logic
├── *.c                     # Motion control and hardware drivers
├── *.h                     # Function prototypes and definitions

## Control Logic Overview

* Robot motion is driven by **hardware timer interrupts**
* Each walking step is decomposed into multiple servo motion groups
* Servos are controlled through the PCA9685 using I2C
* Button inputs are used to switch motion states and adjust speed
* Walking speed is modified by changing timing parameters

---

## How to Build / Run

1. Open the project in **MPLAB X**
2. Select device **PIC24FJ64GA002**
3. Compile using the **XC16 compiler**
4. Flash the firmware to the microcontroller
5. Power the robot and use buttons to control movement

---

## Future Improvements

* Smoother gait transitions
* Turning and backward walking
* Sensor integration (e.g., ultrasonic, IMU)
* Migration to a more powerful MCU or higher-level controller

---

## License

This project is licensed under the **MIT License**.

---

## Author

Yangxu Zhang
Electrical Engineering Student
University of Minnesota Twin Cities
