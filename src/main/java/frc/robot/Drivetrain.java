package frc.robot;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;

public class Drivetrain {

    private DifferentialDrive ddrive;
    private SpeedControllerGroup left;
    private SpeedControllerGroup right;
    private double kp = 0.01;
    
    private Spark lightPower;
    private JeVoisInterface ji;

    private final double visionPower = 0.2;
    private final double visionTurnOffset = 0.03;
    private final int visionRobotCenterPosition = 192 * JeVoisInterface.VIDEO_SCALE;
    private final int visionRobotCenterError = 3 * JeVoisInterface.VIDEO_SCALE;
    private final int visionRobotHorizontalCenter = 120 * JeVoisInterface.VIDEO_SCALE;
    private final int visionRobotHorizontalCenterError = 10 * JeVoisInterface.VIDEO_SCALE;
    private final double sonarStopDistnace = 40.0; // in inches


    public double setpoint = 180.0;

    public Drivetrain(int frontLeft, int rearLeft, int frontRight, int rearRight) {
        left = new SpeedControllerGroup(new WPI_TalonSRX(frontLeft), new WPI_TalonSRX(rearLeft));
        right = new SpeedControllerGroup(new WPI_TalonSRX(frontRight), new WPI_TalonSRX(rearRight));
        ddrive = new DifferentialDrive(left, right);
         
        lightPower = new Spark(0); 
        lightPower.set(1.0);

        ji = new JeVoisInterface(true);
    }

    public void drive(double leftVelocity, double rightVelocity, int exit_or_enter) {
        if(exit_or_enter == -1) {
            ddrive.tankDrive(leftVelocity, rightVelocity, false);
        } else if(exit_or_enter == 0) {
            ddrive.tankDrive(0.4, 0.4);
        } else {
            ddrive.tankDrive(-0.4, -0.4);
        }
    }

    public void turnToAngle(double error) {
        ddrive.tankDrive(kp * error, -kp * error, false);
    }

    // targetSelectionHint could be -1 for left target, 0 middle, 1 right
    // it is just a hint if vision see fewer targets it would just drive
    // to it
    public void driveVision(int targetSelectionHint, VideoTilt vtilt, Sonar sonar) {
        VisionTarget vt = null;
        ArrayList<VisionTarget> targets = ji.getVisionTargets();
        if (targets != null) {
            int numTargets = targets.size();
            // middle target hint
            if (targetSelectionHint == 0) {
                if (numTargets == 3) {
                    // 3 targets; middle hint
                    vt = targets.get(1);
                } else if (numTargets == 2) {
                    // 2 targets; middle hint pick up left one
                    vt = targets.get(0);
                } else {
                    // 1 target
                    vt = targets.get(0);
                }
            } else if (targetSelectionHint == -1) {
                // in all cases
                vt = targets.get(0);
            } else if (targetSelectionHint == 1) {
                if (numTargets == 3) {
                    // 3 targets; right hint
                    vt = targets.get(2);
                } else if (numTargets == 2) {
                    vt = targets.get(1);
                } else {
                    vt = targets.get(0);
                }
            }

            if (vt != null) {
                int center = vt.x;
                boolean stopDistance = false;
                if (sonar != null) {
                    double distance = sonar.getDistance();
                    // System.out.println("distance = " + distance);
                    if (distance < sonarStopDistnace) {
                        stopDistance = true;
                    }
                }

                if (!stopDistance) {
                    if (center < (visionRobotCenterPosition - visionRobotCenterError)) {
                        // drive forward, slightly turn right
                        double offset = visionTurnOffset;
                        // double error = visionRobotCenterPosition - center;
                        // double offset = visionTurnOffset + 
                        // visionTurnOffset * (error / (JeVoisInterface.STREAM_WIDTH_PX * JeVoisInterface.VIDEO_SCALE) / 2);
                        drive(visionPower - offset, visionPower + offset, -1);
                    } else if (center > (visionRobotCenterPosition + visionRobotCenterError)) {
                        // drive forward, sligtly turn left
                        double offset = visionTurnOffset;
                        // double error = center - visionRobotCenterPosition;
                        // double offset = visionTurnOffset + 
                        // visionTurnOffset * (error / (JeVoisInterface.STREAM_WIDTH_PX * JeVoisInterface.VIDEO_SCALE) / 2);
                        drive(visionPower + offset, visionPower - offset, -1);
                    } else {
                        // we are right on target, drive forward
                        drive(visionPower, visionPower, -1);
                    }
                    if (vtilt != null) {
                        int vcenter = vt.y;
                        if (vcenter > visionRobotHorizontalCenter + visionRobotHorizontalCenterError) {
                            vtilt.down();
                        } else if (vcenter < visionRobotHorizontalCenter - visionRobotHorizontalCenterError) {
                            vtilt.up();
                        }
                    }
                } else {
                    // stop we are too close
                    drive(0, 0, -1);
                }
            }
        } else {
            // stop no vision targets
            DriverStation.reportWarning("driveVision: No targets", false);
            drive(0, 0, -1);
        }
    }
}