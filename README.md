# Team 5827 robot code for 2023 - CHARGED UP </br>

# Developers

| Name   | Github Alias    |
|--------|-----------------|
| Ishan  | gamedoescool    |
| Omkar  | datboi212github |
| Tejas  | SanixWheee      |
| Brendan| someonecantcode |
| Satvik | magicgaming58   |
| Swara  | swaraseshadri   |
| LucasCo| piplupsmmm      |
| Max    | MoonLord2436    |
| LucasCa| LucasC351       |
| Jayden | fluffyball9     |
| Matthew| SyndaxError     |
| Leo    | leo5827         |
| Bryan  | zihhsieh        |

-------------------

**Prerequisites**
----
In order to compile, there are a number of prerequisites:

WPILib and VSCode:
https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

CTRE Phoenix library must be installed and enabled as an offline vendor dependency:
https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
</br>

Other vendor dependencies need to be installed using WPILib's online install in VSCode. Click WPI button, then choose "Manage Vendor Libraries", then "Install new libraries (online)" and enter the following URLs:

CTRE Phoenix libraries (if not installed with full install package above)
https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json

REV Robotics (Spark Max and NEO, [Code Release](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.1.1)):
https://software-metadata.revrobotics.com/REVLib-2023.json

PathPlannerLib:
https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json

navX-MXP (Gyro): 
https://dev.studica.com/releases/2023/NavX.json

</br>

# Connecting to the Robot

The robot's WIFI ID is **5827**. Connect your laptop's Wifi to **5827**.

## Connecting to Limelight.

Open a browser and point it to `http://10.58.27.11:5801/`.

# Hardware Setup

You don't need to these, unless you're dealing with old hardware, or want to reuse hardware. There are here for reference.

## Flashing RoboRIO

1. PLUG THE USB-A CABLE INTO THE USB PORT AND CONNECT TO COMPUTER
2. OPEN THE ROBORIO IMAGING TOOL
3. PUT TEAM NUMBER(5827) IN THE TEAM NUMBER BOX
4. CONNECT TO ROBORIO
5. SELECT DESIRED IMAGE IN THE SELECT IMAGE BOX
6. SELECT THE REFORMAT BUTTON AND WAIT UNTIL ROBORIO IS COMPLETELY FLASHED
YOU HAVE SUCCESSFULLY FLASHED A ROBORIO
FOR MORE INFORMATION GO TO https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html

## Flashing and Programming Wireless Radio (OM5P-AN)

The code set of instructions can be found here: https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-3/radio-programming.html

If you're having trouple flashing the Robo RIO, these instructions can be used to flash the firmware manually: https://www.chiefdelphi.com/t/frc-radio-configuration-application-bug-error-finding-npf-device-name-for-adapter-solved/339143

Note:
- Disabling all network drivers except the one you're going to use to flash or configure the device really helps. Simple disable everything else through the Device Manager.
- If you're going to flash the device, consider doing through the command-line (as described in the link). Simply:
  - Open a command-line
  - Go to `C:\Program Files (x86)\FRC Radio Configuration Utility`
  - Run `ap51-flash.exe` to get a list of network adapters
  - Figure out the number of the ethernet adapter you're going to use. A good trick is to run the tool twice, one with the adapter enabled, one disabled, to see what changes.
  - Disconnect the ethernet cable to the radio/device.
  - Run `ap51-flash.exe <adapter-no> <firmware-file>` (e.g. `ap51-flash.exe 3 firmwareOM5PAN.bin`).
  - Reconnect the ethernet cable. Once the device boots up the program should start flashing the new firmware.



**Following info is for 2022 so may be out of date**
----
Setting Constants for Swerve with SDS MK3
----
The following things must be adjusted to your robot and module's specific constants in the Constants.java file (all distance units must be in meters, and rotation units in radians):</br>
1. Gyro Settings: ```invertGyro``` (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. ```trackWidth``` (Center to Center distance of left and right modules)
3. ```wheelBase``` (Center to Center distance of front and rear module wheels)
4. ```wheelDiameter```
5. ```driveGearRatio``` (for SDS MK3 either: (8.16 / 1) or (6.86 / 1))
6. ```angleGearRatio``` (for SDS MK3: (12.8 / 1))
7. Angle Motor PID Values:
    * To tune start with a low P value (0.01).
    * Multiply by 10 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module overshoots the setpoint but corrects with no oscillation.
    * Repeat the process for D. The D value will basically help prevent the overshoot. Ignore I.
8. Get the drive characterization values (KS, KV, KA) by using the WPILib sysid tool. You will need to lock or align your modules straight forward, and complete the characterization as if it was a standard tank drive.  NOTE: The tool didn't behave well for us in 2022 (they were way off). We picked values that seemed theoretically reasonable instead.
9. Tune drive kP until it doesn't overshoot and doesnt oscilate around a target velocity.
10. For ```maxSpeed``` and ```maxAngularVelocity``` you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.
11. Set ```canCoderInvert``` and ```angleMotorInvert``` such that both are CCW+.
12. In the module specific constants, set the can ID's of the motors and CANCoders for the respective modules.
13. Setting Offsets
    * For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight. 
    * You need to point the bevel gears of all the wheels in the same direction (either facing left or right). And preferably you should have the wheels facing in the direction where a postive input to the drive motor drives forward. If for some reason you set the offsets with the wheels backwards, you can change the ```driveMotorInvert``` to fix.
    * Open smartdashboard (or shuffleboard and go to the smartdashboard tab), you will see 4 printouts called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```angleOffset``` variable in constants.


**Controller Mappings**
----
This code is natively setup to use an xbox controller to control the swerve drive. </br>
The Left Stick controls translation (forwards and sideways movement), and the Right Stick controls rotation. </br>
The left bumper button is mapped to zero the gyro and is useful if the gyro drifts during a match.

