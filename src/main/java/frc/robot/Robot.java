/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.*;

import com.ctre.phoenix.motorcontrol.*;

import com.ctre.phoenix.motorcontrol.can.*;


public class Robot extends TimedRobot {

  private HatchGrabber hatchGrabber;
  private DriveControlManager dcm;
  private Drivetrain dtrain;
  private Spark ringLight;
  private UsbCamera front, back;
  private TalonSRX arm = new TalonSRX(1);
  private Spark led = new Spark(1);
  private Cargo cargoGetter;
  private int ktimeoutMs = 10;
  private double pwm = 0, prevV = 0;
  private int targetPos = 600;
  private boolean reverse = false;

  Joystick j = new Joystick(2);

  @Override

  public void robotInit() {
    dcm = new DriveControlManager(); // DCM contains all driver configurations and input
    dtrain = new Drivetrain(12, 11, 10, 9); // initialize drivetrain with given TalonSRX indices
    hatchGrabber = new HatchGrabber(0, 1); // initialize Hatch Grabber
    cargoGetter = new Cargo(2, 3); //init Cargo Grabber
    front = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed
    back = CameraServer.getInstance().startAutomaticCapture(); // give dashboard camera feed

    /* Factory default hardware to prevent unexpected behavior */
    arm.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                      Constants.kPIDLoopIdx, 
                      Constants.kTimeoutMs);
                      arm.setSensorPhase(true); 

    // /* Set relevant frame periods to be at least as fast as periodic rate */
    arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    // /* Set the peak and nominal outputs */
    arm.configNominalOutputForward(0, Constants.kTimeoutMs);
    arm.configNominalOutputReverse(0, Constants.kTimeoutMs);
    arm.configPeakOutputForward(1, Constants.kTimeoutMs);
    arm.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    arm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    arm.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    arm.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    arm.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    arm.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    arm.configMotionCruiseVelocity(80, Constants.kTimeoutMs);
    arm.configMotionAcceleration(100, Constants.kTimeoutMs);

    /* Zero the sensor */
    arm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }

  public void teleopPeriodic() {

    if(dcm.shouldReverse()){
      reverse = !reverse;
    }

    if(!reverse){
      dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldEnterOrExit());
    }else{
      dtrain.drive(-dcm.getRightVelocity(), -dcm.getLeftVelocity(), dcm.shouldEnterOrExit());
    }

    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    if(dcm.shouldGetBall()) {
      cargoGetter.getBall();
    }else if(dcm.shouldReleaseBall()) {
      cargoGetter.releaseBall();
    }

    pwm = j.getRawAxis(3);
    double currV = arm.getSelectedSensorVelocity();
    double accel = (currV - prevV) / 0.02;
    prevV = currV;

    if (arm.getSelectedSensorPosition() > 1200)
      pwm = 0;

   

    if(j.getRawButton(4)){
      arm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    if(j.getRawButton(1)){
      targetPos = 600;
    }
    else if(j.getRawButton(2)){
      targetPos = 750;
    }
    else if(j.getRawButton(3)){
      targetPos = 900;
    }
    else{
      arm.set(ControlMode.PercentOutput, dcm.getArmVelocity());
      return;
    }

    SmartDashboard.putNumber("position", arm.getSelectedSensorPosition());
    SmartDashboard.putNumber("velocity", arm.getSelectedSensorVelocity());
    SmartDashboard.putNumber("acceleration", accel);
    SmartDashboard.putNumber("error", arm.getClosedLoopError() );
    SmartDashboard.putNumber("pwm", arm.getMotorOutputPercent());
    arm.set(ControlMode.PercentOutput, pwm);
    
    int kMeasuredPosHorizontal = 750; //Position measured when arm is horizontal 
    
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    int currentPos = arm.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; 
    double radians = java.lang.Math.toRadians(degrees); 
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.3049;
    double feedFwdTerm = maxGravityFF * cosineScalar;
    arm.set(ControlMode.Position, targetPos, DemandType.ArbitraryFeedForward, feedFwdTerm);
    // arm.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
  }

  public void autonomousInit() {
        // moveOffHab.start();
        hatchGrabber.release();
  }

  public void autonomousPeriodic() {
    if(dcm.shouldReverse()){
      reverse = !reverse;
    }

    if(!reverse){
      dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldEnterOrExit());
    }else{
      dtrain.drive(-dcm.getRightVelocity(), -dcm.getLeftVelocity(), dcm.shouldEnterOrExit());
    }

    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    if(dcm.shouldGetBall()) {
      cargoGetter.getBall();
    }else if(dcm.shouldReleaseBall()) {
      cargoGetter.releaseBall();
    }

    pwm = j.getRawAxis(3);
    double currV = arm.getSelectedSensorVelocity();
    double accel = (currV - prevV) / 0.02;
    prevV = currV;

    if (arm.getSelectedSensorPosition() > 1200)
      pwm = 0;

   

    if(j.getRawButton(4)){
      arm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }
    if(j.getRawButton(1)){
      targetPos = 600;
    }
    else if(j.getRawButton(2)){
      targetPos = 750;
    }
    else if(j.getRawButton(3)){
      targetPos = 900;
    }
    else{
      arm.set(ControlMode.PercentOutput, dcm.getArmVelocity());
      return;
    }

    SmartDashboard.putNumber("position", arm.getSelectedSensorPosition());
    SmartDashboard.putNumber("velocity", arm.getSelectedSensorVelocity());
    SmartDashboard.putNumber("acceleration", accel);
    SmartDashboard.putNumber("error", arm.getClosedLoopError() );
    SmartDashboard.putNumber("pwm", arm.getMotorOutputPercent());
    arm.set(ControlMode.PercentOutput, pwm);
    
    int kMeasuredPosHorizontal = 750; //Position measured when arm is horizontal 
    
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    int currentPos = arm.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; 
    double radians = java.lang.Math.toRadians(degrees); 
    double cosineScalar = java.lang.Math.cos(radians);

    double maxGravityFF = 0.3049;
    double feedFwdTerm = maxGravityFF * cosineScalar;
    arm.set(ControlMode.Position, targetPos, DemandType.ArbitraryFeedForward, feedFwdTerm);
    // arm.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
  }

}
//parth was here