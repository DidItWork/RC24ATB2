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
    1. Forward Backward
    1. **EMPTY**
    1. Left Right
    1. Rock Claw
    1. Lift
    1. Intake In
    1. Intake Out 
    1. Flag Claw
    1. **EMPTY**
- PS4 Controller
    1. Left Joystick Y : Lift
    1. R1 and L1 : Flag Claw
    1. R2 and L2 : Intake
