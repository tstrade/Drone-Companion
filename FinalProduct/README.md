# Drone Companion - Prototype Setup and Functionality
***
## Setup:
1. Load Drone-Companion/FinalProduct/MasterCode/MasterCode.ino into the Arduino IDE
2. Connect Mini-B USB cable from laptop to Arduino Nano USB port and upload the code (nonstandard missing libraries can be found in the Drone-Companion repository)
3. Place the drone on a flat spot (ground, chair, table) and then upload the sketch
4. When the serial monitor instructs you to do so, plug the LiPo battery into the PDB
5. The motors will make a few short beeps and then one long beep to confirm the motors have been armed, then press the IR remote's ON button

***
## How to Use the Drone Companion:
- After the Drone Compion has been setup, it will initiate its takeoff protocol
  - To complete the takeoff protocol, the Drone Companion must be raised to a height 1 meter (approx. 3 feet) above its starting position
 
- After the takeoff protocol, the Drone Companion will assume its hover protocol
    - *Using the IR Remote*
        - The code received by the sensor starts from 0 (top left button), followed by 1, 2, 3; the next row will start on code 4, and so on
        - Using the codes:
            - **Code 1**: Hover protocol
            - **Code 2**: Takeoff protocol
            - **Code 3**: Landing protocol
            - **Other Codes**: Follow protocol
  
    - *Protocols*
        1. **Hover**: maintain a stable / constant height
        2. **Takeoff**: increase motor speed until 1 meter higher than the starting position
        3. **Land**: reduce drone speed over ~30 second period and then stop motors completely
        4. **Follow**: maintain distance from user by moving forwards or backwards
         
    - *Special Behavior*
        - **Hover** will not respond to user distance or obstacle proxomity
        - **Takeoff** can only occur when the drone is landed. If the takeoff protocol is activated while the drone is airborn, it will skip the takeoff sequence and default to the hover protocol
        - **Land** can only occur when the drone is airborn. If the landing protocol is activated while the drone is grounded, nothing will happen.
        - **Follow** will trigger the alarm if an obstacle comes too close to the drone and will only silence when the obstalce has moved away

***
## Notes:
- *LiPo Battery Life* - A fully charged 2200 mAh will allow the drone to run for approximately 6 to 10 minutes continuously before it needs to be recharged.
  
- *Calibration Sequences* - The Drone Companion should not be moved during setup, as it is taking time to calibrate the gyroscope. Additionally, the battery should not be plugged in nor the IR remote used sooner than indicated. This could cause an improper setup and remove any guarantee that the Drone Companion will function as expected.
    - You can find our version of a motor / ESC calibration sequence in "Drone-Companion/Software/Test Code"
 
- *Project Replication* - If you are building your own version, make sure to calibrate the motors to their specified minimum and maximum throttle in a sketch separate from the rest of the hardware. This usually only needs to happen once, but improper calibration or power malfunctions could reset this process. Make sure to follow the manufacturer's specifications. Additionally, PID control constants and complementary filters are NOT universal: these parameters are sensitive and should be tested extensively with the hardware you choose to use.
    - Research the hardware you are using. Understanding the units of the inputs and outputs is vital to proper testing. Additionally, some components may interfere with one another. For example, in this implementation, the IR sensor and buzzer use the same clock timer from the Arduino Nano - such a resource must be mutually exclusive
  
- *Testing Environment* - To avoid broken parts and/or injury, conduct initial testing without propellors. Use the Serial Monitor and visual/auditory queues to debug the motors's reaction to stimulus. When appropriate, choose a safe space to tie at least two of the motor props to heavy objects. It should be high enough off the ground that it will not crash in the event of an error, and tied in such a way that it cannot cut itself loose. Motors are powerful - make sure whatever you secure the drone with will not break from the force created by the drone. Finally, motors rotate over a hundred times per second, so propellors are essentially mini blades if you are not careful. Wear protective clothing and glasses while testing your drone.
 
***
## Final Remarks:
  In the timespan of ~3 months (as of Dec. 6, 2024), this drone has gone from an idea to a functional prototype. This prototype has not yet achieved full flight, due to complications such as the ones we have warned against above, as well as the limited resource that is time. Moving forward, there are two more major phases planned for the Drone Companion:
    - Phase 4 will include a remastering of the drone - a new frame, reorganized wiring, memory optimization, parameter refinement, and sensible testing. The goal of this phase is to have the drone successfully complete its takeoff, hover, and landing protocols.
    - Phase 5 will result in the final product, in which the drone will be able to enter its follow protocol and operate in a smooth, controlled manner.

  If you are seeking to build your own drone using an Arduino, or another microcontroller, it is important to not be discouraged. Integrating sensitive hardware requires careful and extensive testing, and lots (and we mean a lot) of failure. Many online forums, such as the Arduino Forum and Reddit, discourage programmers new to microcontrollers to not bother with a drone. Of course, there are also many useful tips out there on the internet - you just have to look around hard enough to find useful information that applies to you. Hopefully, this project will stand as a testimate to the power of determination and the pursuit of knowledge. 

Sincerely, 
Two undergraduates with a vision

