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
  
  private HatchGrabber hatchGrabber;
  private boolean closed;
  private DriveControlManager dcm;
  private Drivetrain dtrain;
  private Arm arm;

  private AnalogPotentiometer pot = new AnalogPotentiometer(0, 270, 0);

  @Override
  public void robotInit() {
    
    dcm = new DriveControlManager();
    dtrain = new Drivetrain(11, 12, 9, 10);
    arm = new Arm(1);
   

    CameraServer.getInstance().startAutomaticCapture();
  }

  boolean timing = false;
  public void teleopPeriodic() {

    dtrain.drive(dcm.getLeftVelocity(), dcm.getRightVelocity(), dcm.shouldExit());
    arm.actuate(dcm.getArmVelocity(), dcm.shouldFreezeArm());

    if(dcm.shouldGrab()) {
      hatchGrabber.grab();
    } else if(dcm.shouldRelease()) {
      hatchGrabber.release();
    }

    dcm.updateSquat();
  }
  
}
//parth was here