/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.cscore.*;

public class Robot extends TimedRobot {
  
  private HatchGrabber hatchGrabber;
  private DriveControlManager dcm;
  private Drivetrain dtrain;
  private Arm arm;
  private Spark ringLight;

  private Timer moveOffHab;

  private UsbCamera front, back;

  private double howLongShouldWeMove;
  @Override
  public void robotInit() {
    howLongShouldWeMove = 3.0;
    moveOffHab = new Timer();
    dcm = new DriveControlManager(); // DCM contains all driver configurations and input
    dtrain = new Drivetrain(12, 11, 10, 9); // initialize drivetrain with given TalonSRX indices
    arm = new Arm(1); // initialize Arm with TalonSRX index
    hatchGrabber = new HatchGrabber(0, 1); // initialize Hatch Grabber

    front = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed
    back = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed
  }

  public void teleopPeriodic() {

    dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldEnterOrExit());
    // arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());
    arm.pidActuate(-11);
    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    double sp = 50.0;

    SmartDashboard.putNumber("angle", arm.getAngle());
    SmartDashboard.putNumber("setpoint", sp);
    SmartDashboard.putNumber("Error", sp - arm.getAngle());

    // System.out.println(arm.getAngle());

    // System.out.println(dtrain.getCenter());

    // ringLight.set(dcm.getArmVelocity());

    dcm.updateSquat();
  }

  public void autonomousInit() {
    moveOffHab.start();
    hatchGrabber.grab();
  }

  public void autonomousPeriodic() {
    dtrain.drive(dcm.getLeftVelocity() * 0.5, dcm.getRightVelocity() * 0.5, dcm.shouldEnterOrExit());
    arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());

    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    // ringLight.set(dcm.getArmVelocity());

    dcm.updateSquat();
  }
}
//parth was here