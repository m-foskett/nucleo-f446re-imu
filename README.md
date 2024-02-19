# nucleo-f446re-imu

Measuring IMU sensor values from an MPU6050 and displaying them on an LCD1602 module.

Running FreeRTOS to schedule multiple tasks (measure and display) on an STM32 Nucleo-64 development board (F446RE).

Wrote my own library functions to initialise and utilise the devices using the respective register sheets as reference.

![Maintenance](https://img.shields.io/maintenance/yes/2024)
![GitHub top language](https://img.shields.io/github/languages/top/m-foskett/nucleo-f446re-imu)
![GitHub last commit](https://img.shields.io/github/last-commit/m-foskett/nucleo-f446re-imu)

## :scroll: Table of Contents
- [Screenshots](#computer-screenshots)
- [Tech Stack](#books-tech-stack)

## :computer: Screenshots
<div style="display: inline_block" align="center"><br>
 <img align="center" alt="Circuit Diagram"  width="800" src="/screenshots/circuit_diagram.PNG">
   <img align="center" alt="Initialisation"  width="400" src="/screenshots/initialisation.gif">
 <img align="center" alt="Demo Usage"  width="400" src="/screenshots/demo_usage.gif">
</div>

## :books: Tech-Stack
Below is a non-exhaustive list of the technologies utilised within this project.
| Devices | Languages | Libraries |
| ----------- | ----------- | ----------- |
| MPU6050 IMU Device | Embedded C | FreeRTOS |
| LCD1602 Module |  | |
| STM32 Nucleo-64 F446RE | | |
