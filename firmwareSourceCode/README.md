# Driver for Smart Sensing Thread Factory

## Overview

This project is an Arduino-based system that integrates multiple sensors, stepper motors, and a heating element. It monitors temperature, tension, and conductivity using sensors and provides control mechanisms through stepper motors and a heating element. A PID controller is implemented for precise tension regulation, and real-time data is displayed on an LCD. User inputs allow dynamic adjustment of system parameters.

### Key Features:
- **Temperature Monitoring & Control**: Uses a thermistor to measure temperature and adjust a heater to maintain a target temperature.
- **Stepper Motor Control**: Drives three stepper motors with adjustable speed and direction to manage processes like ink feeding and tension control.
- **PID Control**: Ensures the tension rate is maintained within the desired range using feedback loops.
- **LCD Display**: Real-time display of key system parameters like temperature, tension, conductivity, and motor speed.
- **Button Inputs**: Four buttons allow users to adjust system settings, start/stop processes, and initiate thread tests.
- **Sensors**:
  - **Thermistor** for temperature.
  - **Tension Sensor** for tension measurement.
  - **Conductivity Sensor** for measuring conductivity.

## Hardware Components

1. **Arduino Board** (e.g., Arduino Uno)
2. **LCD Display**: I2C 20x4 display
3. **Thermistor**: For temperature sensing
4. **Conductivity Sensor**: To measure conductivity
5. **Tension Sensor**: To measure the tension rate
6. **Stepper Motors**: To control the mechanical processes (3 stepper motors)
7. **Buttons**: Four buttons for input control
8. **Heater**: To adjust and maintain temperature
9. **Fan**: To assist in cooling (optional)
10. **Wiring and Power Supplies**

### Pin Configuration

| Component                 | Pin Number    |
|---------------------------|---------------|
| **Thermistor Pin**         | A6            |
| **Fan Pin**                | 6             |
| **Conductivity Sensor Pin**| A2            |
| **Tension Sensor Pin**     | A3            |
| **Heater Pin**             | 9             |
| **Stepper Motor A**        | Dir: 2, Step: 3 |
| **Stepper Motor B**        | Dir: 7, Step: 8 |
| **Pump Motor**             | Dir: 10, Step: 11 |
| **LCD I2C Address**        | 0x27          |

## Software Setup

### Requirements

- **Arduino IDE** (Download from [Arduino](https://www.arduino.cc/en/software))
- **Libraries**: 
  - `Wire.h`
  - `LiquidCrystal_I2C.h`

### Installing Libraries

1. Open the Arduino IDE.
2. Navigate to `Sketch > Include Library > Manage Libraries...`.
3. Search for **LiquidCrystal_I2C** in the Library Manager.
4. Install the **LiquidCrystal_I2C** library.

### Code Compilation and Upload

1. **Open the Code**:
   - Copy the provided Arduino code into a new sketch in the Arduino IDE.

2. **Select Your Arduino Board**:
   - Go to `Tools > Board` and select your Arduino model (e.g., Arduino Uno).

3. **Select the Correct Port**:
   - Go to `Tools > Port` and select the port your Arduino is connected to.

4. **Verify and Upload**:
   - Click the checkmark button (`Verify`) to compile the code.
   - Click the arrow button (`Upload`) to upload the compiled code to your Arduino.

5. **Connect the Components**:
   - Wire the thermistor, tension sensor, conductivity sensor, stepper motors, heater, and LCD according to the pin configuration above.

### Usage Instructions

- **Adjust Parameters**: Use the buttons to adjust temperature, motor speed, and tension rate as needed.
- **Start/Stop Processes**: Use the buttons to start/stop coating processes and initiate thread tests.
- **Monitor the LCD**: View real-time sensor readings, including temperature, tension rate, conductivity, and motor speed on the LCD display.
  
### Button Controls:

| Button   | Function                              |
|----------|---------------------------------------|
| **Up**   | Navigate up in the menu               |
| **Down** | Navigate down in the menu             |
| **Left** | Decrease the selected parameter       |
| **Right**| Increase the selected parameter       |

### LCD Display Output:
- **Temperature**: Current temperature and target temperature
- **Conductivity**: Current conductivity value
- **Tension Rate**: Current and desired tension rate
- **Motor Speed**: Speed of the stepper motors

## Serial Monitoring

To monitor data and debug via the Serial Monitor:
1. Open **Serial Monitor** from `Tools > Serial Monitor` in the Arduino IDE.
2. Set the baud rate to **115200** to match the code.

The serial monitor will display:
- Stepper motor positions
- Tension rate and conductivity
- Test cycle results during thread tests

## Troubleshooting

- **No LCD Output**: Ensure the LCD is properly connected to the correct I2C address (`0x27`), and the contrast is adjusted.
- **Stepper Motors Not Running**: Check the wiring and verify that the stepper motors are connected to the correct pins.
- **Temperature Control Issues**: Ensure the thermistor is wired correctly and is within the operational range.
- **Buttons Not Responding**: Check the debouncing logic and verify correct pin connections.

## License

This project is open-source. Feel free to modify and distribute as needed.
