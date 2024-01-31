### Libraries Used

- [PS4-esp32](https://github.com/aed3/PS4-esp32.git)
- [iBusBM](https://github.com/bmellink/IBusBM)
- [arduino-esp32 v2](https://github.com/espressif/arduino-esp32/tree/release/v2.x)

### Software Used

- [SixaxisPairTool 0.3.1](https://www.filehorse.com/download-sixaxispairtool/)

### Programs

- [getBluetoothAddresss](getBluetoothAddress/getBluetoothAddress.ino)

    Get's the bluetooth address of the esp32. The MAC address the PS4 Controller stores will then be changed to this address with *SixaxisPairTool*.

- [RC24_Mec_and_Servo_RC](RC24_Mec_and_Servo_RC/RC24_Mec_and_Servo_RC.ino)

    The main program ran on the robot's ESP32 board to accept controls from both the FlySky Controller and PS4 Controller.


### Channel Mappings

- FlySky Controller
    1. Pivot Turn
    2. Forward Backward
    3. **EMPTY**
    4. Left Right
    5. Rock Claw
    6. Lift
    7. Intake In
    8. Intake Out 
    9. Flag Claw
    10. **EMPTY**
- PS4 Controller
    1. Left Joystick Y : Lift
    2. R1 and L1 : Flag Claw
    3. R2 and L2 : Intake
