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

***
# **Accelerometer**

***
# **Power Distribution Board**

***
# **Motors**

***
# **Buzzer**

***
# **IR Transmitter/Receiver**

