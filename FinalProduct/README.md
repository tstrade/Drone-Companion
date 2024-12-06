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
        1. **Hover**: maintain a stable / constant height; will not follow user or warn of obstacles that are too close
        2. **Takeoff**: increase motor speed until 1 meter higher than the starting position
        3. **Land**: reduce drone speed over ~30 second period and then stop motors completely
        4. **Follow**: maintain distance from user by moving forwards or backwards; also, trigger alarm if an obstacle is too close
     
    - *Special Behavior*
        - **Takeoff** can only occur when the drone is landed. If the takeoff protocol is activated while the drone is airborn, it will skip the takeoff sequence and default to the hover protocol
        - **Land** can only occur when the drone is airborn. If the landing protocol is activated while the drone is grounded, nothing will happen.
     
***
## Notes:

