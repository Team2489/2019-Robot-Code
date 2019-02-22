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

public class Robot extends TimedRobot {
  
  private DoubleSolenoid hatchGrabber = new DoubleSolenoid(0,1);
  
  private boolean closed;
  private DriveControlManager dcm;
  private Drivetrain dtrain;
  private Arm arm;

  
  private AnalogPotentiometer pot = new AnalogPotentiometer(0, 270, 0);

  @Override
  public void robotInit() {
    closed = false;
    
    dcm = new DriveControlManager();
    dtrain = new Drivetrain(11, 12, 9, 10);
    arm = new Arm(1);
   

    CameraServer.getInstance().startAutomaticCapture();
  }

  Timer hatchTimer = new Timer();
  boolean timing = false;
  public void teleopPeriodic() {
    dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldExit());
    arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());
    if((dcm.shouldGrab() && closed) && !timing){
      closed = false;
      timing = true;
      hatchTimer.start();
      hatchGrabber.set(DoubleSolenoid.Value.kReverse);
    }else if((dcm.shouldRelease() && !closed) && !timing){
      closed = true;
      timing = true;
      hatchTimer.start();
      hatchGrabber.set(DoubleSolenoid.Value.kForward);
    }

    // Sma.putNumber("Angle", pot.get());
    System.out.println(1);

    dcm.updateSquat();

    if(hatchTimer.get() >= 1) {
      hatchTimer.reset();
      timing = false;
    }
  }
}
//parth was here