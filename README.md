# 2019-Robot-Code

Code located in src/main/java. Do not edit Main.java!

Class Structure:
1. **Robot.java** - Code starts here and contains all of the commands executed in the main loop.
2. **DriveControlManager.java** - DriveControlManager.java (DCM) contains the current state of the drivers including configs and controls
3. **Drivetrain.java** - Genereic drop-in code for a standard 4-motor tank drivetrain with an added exit feature
4. **Arm.java** - Contains arm control code with a feature to limit its acceleration to prevent breakage and to freeze the arm in position
5. **HatchGrabber.java** - Implements simple grab and release functions for the grabber.


Need to Add:
1. Potentiometer and PID for Arm angle sensing and control (Potentiometer code added)
2. Vision Code for drive base alignment with target (Drive Base Alignment code added)
