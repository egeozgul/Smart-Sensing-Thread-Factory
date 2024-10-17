# Smart-Sensing-Thread-Factory

<p align="center">
  <img src="Thread_Factory_Model_render.png" width="1000">
</p>

## Overview

This is a device built for Tufts NanoLab lab to assist the sensor development and wearable electronics research projects. This device automates the smart thread sensor production process using a combination of sensors and actuators.
This project is published in MDPI: [Publication Link](https://www.mdpi.com/2072-666X/15/10/1239) 

## Contents

- **PCB_productionFiles**: This folder contains all the necessary PCB design files required for creating the electronic boards that drive the system. The PCBs connect the microcontroller to the sensors and motors, enabling accurate monitoring and control.
  
- **cartridgeDesignFiles**: Inside this folder, you will find the 3D printable designs for various mechanical components, particularly the thread cartridge system. These parts can be printed using a 3D printer and are crucial for holding and guiding the thread during production.

- **firmwareSourceCode**: This directory includes the firmware for the microcontroller, which handles the overall operation of the system. The firmware reads sensor data, controls the stepper motors, adjusts system settings via a user interface, and maintains optimal conditions using PID control.
