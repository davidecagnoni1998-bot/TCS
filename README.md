# TCS Telescope Control System
Embeddded controller for astronomical telescopes

## Overview
This project is a complete "control station" for my amatorial astronomical telescope.
The system is composed by an hand-sized controller equipped with a touch screen to control the telescopes
and some stepper motors and DC motors installed directly on the telescope and the telescope mount to adjust its pointing and focus
without touching the telescope and thus without creating wobbing and trembling.

## Hardware
- Arduino Mega 2560
- Nextion touch display
- drv8833 as H bridge
- A4988 as stepper driver
- Stepper motors (Nema 17)
- DC motors
- Optical encoder

## Software
Developed using:
- Visual Code 

## Project Structure
In this repo two folders are present, inside "Control module" the code running on the Arduino Mega is present while
in the folde GUI e Nextion file is present.
