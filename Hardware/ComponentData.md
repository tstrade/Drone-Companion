# **Ultrasonic Sensor**
## Electric Parameters:

| Working Voltage | DC 5V |
| :---: | :---: |
| Working Current | 15mA |
| Working Frequency | 40Hz |
| Range | 0.03 - 13.12ft |
| Accuracy | 0.01ft|
| Measuring Angle | 15 degrees |
| Trigger Input Signal | 10us TTL Pulse |
| Echo Output Signal | Input TTL level signal and the range in proportion |

[Transistor-Transistor Logic (TTL)](https://meanwellpowersupplies.com/faq/what-is-a-ttl-signal/#:~:text=A%20TTL%20signal%2C%20Transistor%2DTransistor,5V%20is%20a%20high%20state.)

- Basic Principles:
  1. Use I/O trigger for at least 10us logic HI
  2. Module automatically sends eigth 40kHz and detects whether there is a pulse signal back
  3. If there is a pulse signal back, through logic HI, time of logic HI I/O duration is from time sending ultrasonic to returning
  4. Appropriate test distance = (logic HI time * 1115ft/s)/2
       - where 1115ft/s is approx. the speed of sound (340m/s) in feet
    
- Wire Connections:
    - 5V supply
    - Trigger Pulse Input
    - Echo Pulse Output
    - 0V ground

The timing diagram and all information listed in this section can be found in the Ultrasonic Ranging Module HC-SR04 datasheet included in this repository. Any additional information may be found in the module's [Github Repository](https://github.com/sparkfun/HC-SR04_UltrasonicSensor)


## Dimensions:
- 45 x 20 x 15 millimeters
- 1.8 x 0.8 x 0.6 inches

## Input Data:
- To trigger, send a 10us TTL pulse to the module
  - TTL logic HI:  ~ 2.7 - 5.0V
  - TTL logic LO: ~ 0.0 - 0.4V
  - Note: This information might vary based on our component's capabilities. The TTL link above, as well as others, tend to be product-specific.
 
- Suggested to use a measurement cycle > 60ms to avoid I/O overlap

## Output Data:
- Sends a TTL pulse "with a range in proportion"
- The distance, in inches, of the object that is within the range/width of the sensor is defined as $$d = \frac{t_{o} - t_{i}}{148}$$

***
# **Altitude Sensor**
## Overview
- Barometric Pressure Accuracy: $\pm 1$ hPa
- Temperature Accuracy: $\pm 1.0^\circ$C
- Altimeter Accuracy: $\pm 1$ meter
- Low Altitude Noise: $0.25$ meters
- ! Can be used with 3.3V or 5V systems thanks to built-in voltage regulator

## Pinouts:
| Pins | Use |
| :---: | :---: |
| Vin | Power pin - use same power as logic level of microcontroller |
| 3Vo | 3.3V output from voltage regulator - can grab up to 100mA |
| GND | Common ground for power and logic |
| --------- | --------------------------------------------------------------------------------- |
| ***SPI*** | ***PROTOCOL PINS*** |
| SCK | SPI Clock Pin - input to chip |
| SDO | Serial Data Out / Microcontoller In Sensor Out pin - data sent from sensor to processor |
| SDI | Serial Data In / Microcontroller Out Sensor In pin - data sent from processor to sensor |
| CS | Chip Slect pin - drop to low to start SPI transaction; input to chip |
| --------- | --------------------------------------------------------------------------------- |
| ***I2C*** | ***PROTOCOL PINS*** |
| SCK (SCL) | I2C Clock Pin - connect to microcontroller's I2C clock line |
| SDI (SDA) | I2C Data Pin - connect to microcontroller's I2C data line |
| SDO | Leave unconnected |
| CS | Leave unconnected |

[BMP280 to Arduino Wiring-by-Protocol](https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/arduino-test)

## Input/Output Data:

Adafruit provides a library for the BMP280 that is available from the Arduino library manager, and can be installed from GitHub [here](https://github.com/adafruit/Adafruit_BMP280_Library/tree/master).

Additionally, the GitHub repository includes a few examples on how to interact with the BMP280 through an Arduino sketch. It is encouraged to review this documentation, but the examples can be summarized as follows: initialize sensor, call for a temperature and/or pressure reading using `getEvent()`, and store or print the value obtained by the sensor. 

What to look into during the coding subphase: 
- Different operating modes
- Oversampling values
- Standby timing
- Proper addressing for chosen protocol

In addition to the GitHub examples for library usage, the Adafruit provides a section on [how to use the library with the SPI protocol](https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/arduino-test). 

***
# **Flight Controllers**

| Specifications | Details Found [Here](https://www.rcelectricparts.com/40a-esc---classic-series.html) |
| :----: | :----: |
| Continuous | 40 A |
| Bursts | 60 A for 10 seconds |
| *UBEC | 5V/3A |
| LiPo Battery | 2 - 4 cells |
| **NiDc/NiMH | 5 - 12 cells |
| Dimensions | 71 x 35 x 10.5 mm |
| Weight | 48 g |
| Motor Type | Brushless |
| Max RMP - 2 Pole Motor | 240,000 RPM |
| Max RMP - 6 Pole Motor | 80,000 RPM |
| Max RMP - 12 Pole Motor | 40,000 RPM |

*Universal Battery Eliminator Circuit - converts high-voltage power to lower voltages
**Nickel-cadmium and Nickel-metal hydride are types of rechargeable batteries
  - NiCd has a short charge time and high discharge rate
  - NiMH is less toxic and holds more power

## Input/Output Data:
A complete guide to the GOUPRC 40A Brushless ESC can be found [here](https://www.rcelectricparts.com/classic-esc-user-guide.html).

- Connections
  - Battery: use pre-soldered XT60 plug
  - Motor: 3 wires (opposite the XT60 plug)
      - swap any two connections to reverse motor direction OR
      - program ESC to spin in reverse direction
  - Arduino: black ground, red 5V, white signal wires (next to XT60 plug)
 
- Input Signal
  - PWM Signal Frequency: 50 Hz
  - Zero Throttle: 1 ms pulse width
  - Full Throttle: 2 ms pulse width
  - *Note: PWM signal operates on a spectrum between these endpoints -> allows for incremental control*

*RC Electric Parts provides [this video](https://youtu.be/uOQk8SJso6Q) for Arduino usage*
 
## Procedures:
Calibration
  1. Put throttle in maximum position
  2. Plug in ESC
  3. Wait until ESC starts beeping
  4. While beeping, put throttle in minimum position
  5. Wait for confirmation beeps
  6. Unplug ESC
 
Start up
  1. Put throttle in minimum position
  2. Plug in ESC
  3. ESC will beep: 1x for each detecting cell + 1x if brake ON OR 2x if brake OFF
  4. ESC is ready

## Benefits:
- Smooth, programmable, and accurate linear throttle adjustment
- Supports high RPM motors
- Utilizes smaller MOSFETs to minimize weight
- Minimal heat dissipation
- Pre-soldered with 3.5 mm Bullet Plugs and an XT60 Plug
- Reduces electromagnetic interference signal input
- Beeps during start up to confirm the number of battery cells

## Built-In Safety Features:
- Start-up Protection: will stop powering motor if it detects motor is unable to spin after start up
- Safe Start Protection: will not power the motor on start up unless throttle is in minimum throttle position to prevent dangerous accidental start ups
- Overheat Protection: output power reduces to 40% when over $100^\circ$C ($212^\circ$F)
- Lost Signal Protection: when signal is lost the ESC cutes the motor's power to prevent a runaway object
- Low Voltage Protection: if battery voltage drops below programmed threshold, will shut off or reduce throttle depending on programmed settings

*Note: Need to investigate how some of these criteria are met to avoid false-alarms / self-inflicted bugs*

***
# **Accelerometer / Gyroscope**

| Specifications | Details Found [Here](https://www.haoyuelectronics.com/Attachment/GY-521/mpu6050.pdf) |
| :----: | :----: |
| Main Chip | MPU-6050 |
| Power Supply | 3.3-5 V |
| VDD | 2.375-3.46 V |
| VLOGIC | $1.71 \pm 5%$ OR VDD |
| Operaitng Current | 3.9 mA |
| Communication Mode | I2C Protocol (using Wire Arduino Library) |
| Chip Built-in | 16 bit A/D converter, 16 bit data output |
| Gyroscopic Range | $\pm$ 250, 500, 1000, 2000 degrees/sec |
| Gyroscope Operating Current | 3.6 mA |
| Gyroscope Standby Current | 5 $\mu$A |
| Gyroscope Start up Time | 30 ms |
| Acceleration Range | $\pm$ 2, 4, 8, 16g |
| Accelerometer Operating Current | 500 $\mu$A |
| Accelerometer Low Power Current at 1.25 Hz | 10 $\mu$A |
| Accelerometer Low Power Current at 5 Hz | 20 $\mu$A |
| Accelerometer Low Power Current at 20 Hz | 60 $\mu$A |
| Accelerometer Low Power Current at 40 Hz | 110 $\mu$A |

| Gyro/Accel Axes | 3 (X, Y, Z) |

Important Notes:
- I2C Operating Frequency ranges from 100 to 400 kHz
- IC2 Address is either `AD0 = 0` ($1101000$) or `AD0 = 1` ($1101001$)

## Pinout Summary:
- Pin 8: VLOGIC
- Pin 9: AD0
- Pin 23: SCL
- Pin 24: SDA

## Input Data:
- High Level Input Voltage $V_{IH} = 0.7 \times$ VLOGIC
- Low Level Input Voltage $V_{IL} = 0.3 \times$ VLOGIC
- Gyro sensitivity set by `FS_SEL = 0, 1, 2, or 3` (corresponding values defined in table above)
- Accel sensitivity set by `AFS_SEL = 0, 1, 2, or 3` (corresponding values defined in table above)

## Output Data:
- High Level Output Voltage $V_{OH} = 0.9 \times$ VLOGIC (assuming a load resistor of $R_L = 1M\Omega$
- Low Level Output Voltage $V_{OL} = 0.1 \times$ VLOGIC (assuming a load resistor of $R_L = 1M\Omega$
- Data rate of 8000 Hz (Gyro), 1000 Hz (Accel)
- 1024 byte FIFO buffer
  - Reduces power consumption by allowing host processor to read data in bursts and go into low-power mode as sensor collects more data
- Digital output temperature sensor
- Possibility for gesture recognition??? (page 11 of manual linked above)

***
# **Power Distribution Board**

***
# **Motors**

| Specifications | Details Found [Here](https://speedyfpv.com/products/720-brushed-motor-set-4pcs-7x20mm-coreless-motor-1mm-shaft?variant=44109063946454) |
| :----: | :----: |
| Operating Voltage | 3.7-4.2 V |
| Operating Current | 80-100 mA |
| Speed | 45,000 RPM |
| Motor Diameter | 7 mm |
| Motor Length | 20 mm |
| Propeller Shaft Diameter | 1 mm |
| Propeller Shaft Length | 5 mm |
| Propeller Length | 55 mm |
| Overall Weight | 3g |


***
# **Buzzer**

***
# **IR Transmitter/Receiver**

