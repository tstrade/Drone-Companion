# ESC Notes:
- ESC waits for a low PWM signal to arm itself
- Min and Max PWM values can be calibrated
- Object - ESC name (PIN#, MIN, MAX, ARMING VALUE)
	ex.) `ESC myESC (9, 1000, 2000, 500);`

## Functions:
- `calib()`: Initiate the calibration process
	- Once the LED (LED_PIN) is HIGH/ON connect the power to your ESC, you have 5sec to do so
	- Once the LED is LOW/OFF the calibration should be done
	- ex.) `myESC.calib();`

- `arm()`: sends the arming PWM value to initiate the ESC
	- ex.) `myESC.arm();`

- `stop()`: Sends a 500us value to stop the ESC
	- value can be changed in the library
	- ex.) `myESC.stop();`

- `speed()`: updates ESC speed
	- ex.) `myESC.speed(1100);`

## ESC Library
- [GitHub Repo](https://github.com/RB-ENantel/RC_ESC/archive/master.zip)
	- Can also look up ESC library Arduino and download the latest version on the website.

- In the .zip file there are examples of the following functions:

	- `ESC.calib()`: Usually only done once. Will send a high value at power up and after a delay decreases to a low value. These values will then be saved in the ESC and used for the range of speed

	- `ESC.knob()`: Based on `knob()` in the servo library. It adjusts the ESC speed based on a potentiometer.
			- Example shows ESC responding to an RC controller; however, will likely usecase is to map the ultrasonic sensor values to a speed.

	- `ESC.ramp()`: Based on `sweep()` in the servo library. Ramps up and down the speed on a given range.
 			- Will (mostly) be used for activation / deactivation stages


   
