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


import com.ctre.phoenix.motorcontrol.*;

import com.ctre.phoenix.motorcontrol.can.*;

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

  private BallDispenser ballDispenser;

  private TalonSRX _talon = new TalonSRX(1);

  Joystick j = new Joystick(2);

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
    ballDispenser = new BallDispenser(2, 3);


    /* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();


		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
											Constants.kPIDLoopIdx, 
                      Constants.kTimeoutMs);
    _talon.setSensorPhase(true); 

    /* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);



		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);



    _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(80, Constants.kTimeoutMs);
    _talon.configMotionAcceleration(100, Constants.kTimeoutMs);

    /* Zero the sensor */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    vtilt = new VideoTilt(2);
    // sonar = new Sonar(1);
    sonar = null;

    front = CameraServer.getInstance().startAutomaticCapture(0);
    // front = CameraServer.getInstance().startAutomaticCapture("Microsoft Camera", 0); // give dashboard camera feed
    // front.setVideoMode(PixelFormat.kMJPEG, 640, 320, 15);
    front.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);

    // back = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed
  }

  private double pwm = 0, prevV = 0;
  private int targetPos = 600;

  public void teleopPeriodic() {
    if (dcm.shouldVisionDrive()) {
      // drive based on JeVois info
      dtrain.driveVision(dcm.getVisionHint(), vtilt, null);
    } else {
      // drive on Joysticks
      dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldEnterOrExit());
    }
    
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

    if (dcm.shouldBallDispenserPush()) {
      ballDispenser.push();
    }

    if (dcm.shouldBallDispenserRetract()) {
      ballDispenser.retract();
    }

    double sp = 50.0;

    pwm = j.getRawAxis(3);

    double currV = _talon.getSelectedSensorVelocity();
    double accel = (currV - prevV) / 0.02;
    prevV = currV;

    if (_talon.getSelectedSensorPosition() > 1200)
      pwm = 0;

    // _talon.set(ControlMode.PercentOutput, pwm);

    // if(j.getRawButton(4))
    // _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    if(j.getRawButton(1))
      targetPos = 600;
    else if(j.getRawButton(2))
      targetPos = 750;
    else if(j.getRawButton(3))
      targetPos = 900;
    else
      arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());

    SmartDashboard.putNumber("position", _talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("velocity", _talon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("acceleration", accel);
    SmartDashboard.putNumber("error", _talon.getClosedLoopError() );
    SmartDashboard.putNumber("pwm", _talon.getMotorOutputPercent());
    _talon.set(ControlMode.PercentOutput, pwm);
    
    int kMeasuredPosHorizontal = 750; //Position measured when arm is horizontal 
    
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    int currentPos = _talon.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; 
    double radians = java.lang.Math.toRadians(degrees); 
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.3049;
    double feedFwdTerm = maxGravityFF * cosineScalar;
    _talon.set(ControlMode.Position, targetPos, DemandType.ArbitraryFeedForward, feedFwdTerm);
    

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