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
import edu.wpi.cscore.VideoMode.PixelFormat;

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

  private VideoTilt vtilt;
  private Sonar sonar;


  @Override
  public void robotInit() {
    howLongShouldWeMove = 3.0;
    moveOffHab = new Timer();
    dcm = new DriveControlManager(); // DCM contains all driver configurations and input
    // competion robot
    dtrain = new Drivetrain(12, 11, 10, 9); // initialize drivetrain with given TalonSRX indices
    // tester robot 2222
    // dtrain = new Drivetrain(7, 6, 2, 9);
    arm = new Arm(1); // initialize Arm with TalonSRX index
    hatchGrabber = new HatchGrabber(0, 1); // initialize Hatch Grabber

    vtilt = new VideoTilt(2);
    // sonar = new Sonar(1);
    sonar = null;

    front = CameraServer.getInstance().startAutomaticCapture(0);
    // front = CameraServer.getInstance().startAutomaticCapture("Microsoft Camera", 0); // give dashboard camera feed
    front.setVideoMode(PixelFormat.kMJPEG, 640, 320, 15);
    //front.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);

    // back = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed
  }

  public void teleopPeriodic() {
    if (dcm.shouldVisionDrive()) {
      // drive based on JeVois info
      dtrain.driveVision(dcm.getVisionHint(), vtilt, null);
    } else {
      // drive on Joysticks
      dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldEnterOrExit());
    }
    arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());
    if(dcm.shouldTurnLeft()) {
      dtrain.drive(-0.2 / dcm.k, 0.2 / dcm.k,  dcm.shouldEnterOrExit());
    }

    if(dcm.shouldTurnRight()) {
      dtrain.drive(0.2 / dcm.k, -0.2 / dcm.k,  dcm.shouldEnterOrExit());
    }
    //arm.pidActuate(-11);
    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    vtilt.hold();

    if (dcm.shouldVtiltHorizontal()) {
      vtilt.setHorizontal();
    }
    if (dcm.shouldVtiltUp()) {
      // System.out.println("dmc.shouldVtiltUp");
      vtilt.up();
    }

    if ( dcm.shouldVtiltDown()) {
      // System.out.println("dcm.shouldVtiltDown");
      vtilt.down();
    }

    if (dcm.shouldJevoisHumanMode()) {
      if (dtrain.ji != null) {
        dtrain.ji.setCamHumanDriverModeAsync();
      }
    }

    if (dcm.shouldJevoisVisionMode()) {
      if (dtrain.ji != null) {
        dtrain.ji.setCamVisionProcModeAsync();
      }
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
    hatchGrabber.release();
  }

  public void autonomousPeriodic() {
    //if (dcm.shouldVisionDrive()) {
      // drive based on JeVois info
      // dtrain.driveVision(dcm.getVisionHint(), vtilt, null);
    //} else {
      dtrain.drive(dcm.getLeftVelocity() * 0.5, dcm.getRightVelocity() * 0.5, dcm.shouldEnterOrExit());
    //}
    arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());

    if(dcm.shouldTurnLeft()) {
      dtrain.drive(-0.15 / dcm.k, 0.15 / dcm.k,  dcm.shouldEnterOrExit());
    }

    if(dcm.shouldTurnRight()) {
      dtrain.drive(0.15 / dcm.k, -0.15 / dcm.k,  dcm.shouldEnterOrExit());
    }

    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    vtilt.hold();

    // ringLight.set(dcm.getArmVelocity());

    dcm.updateSquat();
  }
}
//parth was here