# FinalProject

# Drivable Shopping Basket

This repository contains the Arduino code for the Drivable Shopping Basket project. The basket can be controlled using Bluetooth inputs from a joystick, a string, or an integer value.

## Description

The Drivable Shopping Basket is designed to follow commands sent from a Bluetooth-connected device. It uses a magnetometer for heading detection, a GPS module for location tracking, and a motor driver to control the wheels.

## Components

- Microcontroller (Arduino Uno)
- HMC5883L compass module
- NEO-6M GPS Module
- L298N Motor Driver
- Bluetooth Module (HM-1005)
- Jumper wires
- power supply

## Prerequisites

Make sure you have the following installed:

- Arduino IDE (latest version recommended)
- SerialBluetooth app on your smartphone 

## Setup


### Libraries

The project uses the following libraries:

- `Wire.h` (for I2C communication, comes with the Arduino IDE)
- `SoftwareSerial.h` (for serial communication, comes with the Arduino IDE)
- `Adafruit_Sensor.h` (Adafruit unified sensor interface library)
- `Adafruit_HMC5883_U.h` (for HMC5883L compass)

The `Adafruit_Sensor` and `Adafruit_HMC5883_U` libraries are included in the `libraries` folder within this repository. You need to add these libraries to your Arduino IDE before uploading the code to your microcontroller.

#### Installing Libraries

To install the included libraries, follow these steps:

1. Navigate to the `libraries` folder in this repository.
2. Copy the `Adafruit_HMC5883_Unified` and `Adafruit_Unified_Sensor` folders.
3. Paste them into your local `Arduino/libraries` directory.
Or they both can be installed by finding the Adafruit HMC5883 Unified library in library manager and they both will go to the library file within the Arduino file



## Usage

Once the basket is powered on and the code is uploaded:

1. Connect to the Bluetooth module using your phone or computer.
2. Send commands via the Bluetooth terminal:
   - Joystick input as "J0:angle,speed"
   - String commands for direction: "north", "south", "east", "west"
   - Integer values for moving forward in meters.





