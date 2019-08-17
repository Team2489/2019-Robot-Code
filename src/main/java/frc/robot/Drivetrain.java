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
    public JeVoisInterface ji;

    private final double visionPower = 0.25; // 0.25 0.35
    private final double visionTurnOffset = 0.08; // 0.05 0.08
    // lesser movea to right
    private final int visionRobotCenterPosition = 185 * JeVoisInterface.VIDEO_SCALE; // 200//230 // 185
    private final int visionRobotCenterError = 3 * JeVoisInterface.VIDEO_SCALE;
    private final int visionRobotHorizontalCenter = 120 * JeVoisInterface.VIDEO_SCALE;
    private final int visionRobotHorizontalCenterError = 10 * JeVoisInterface.VIDEO_SCALE;
    private final double sonarStopDistnace = 30.0; // in inches


    public double setpoint = 180.0;

    public Drivetrain(int frontLeft, int rearLeft, int frontRight, int rearRight) {
        left = new SpeedControllerGroup(new WPI_TalonSRX(frontLeft), new WPI_TalonSRX(rearLeft));
        right = new SpeedControllerGroup(new WPI_TalonSRX(frontRight), new WPI_TalonSRX(rearRight));
        ddrive = new DifferentialDrive(left, right);
         
        lightPower = new Spark(0); 
        lightPower.set(1.0);

        try {
            ji = new JeVoisInterface(true);
        } catch (Exception ex) {
            ji = null;
        }
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

    private final double maxDistance = 110;
    private final double minDistance = 60;
    private final double gain = 0.3; //0.5 //0.4

    private double getSpeedRate(double distance){
        double ret = 1.0;
        if (distance > maxDistance){
            ret += gain;
        }else if (distance > minDistance){
            ret += ((distance - minDistance) / (maxDistance - minDistance)) * gain;
        } 
        
        return ret;
    }

    // targetSelectionHint could be -1 for left target, 0 middle, 1 right
    // it is just a hint if vision see fewer targets it would just drive
    // to it
    public void driveVision(int targetSelectionHint, VideoTilt vtilt, Sonar sonar ) {
        if (ji != null) {
        VisionTarget vt = null;
        ArrayList<VisionTarget> targets = ji.getVisionTargets();

        if (targets != null) {
            // left target only
            vt = targets.get(0);

            if (vt != null) {
                int center = vt.x;
                double distance = vt.distanceInches();
                double angle = vt.getAngle();
                double speedRate = getSpeedRate(distance);
                if (angle > 5){

                }
                boolean stopDistance = false;
                if (sonar != null) {
                    double sonarDistance = sonar.getDistance();
                    // System.out.println("distance = " + distance);
                    if (sonarDistance < sonarStopDistnace) {
                        stopDistance = true;
                    }
                }

                if (!stopDistance) {
                    if (center < (visionRobotCenterPosition - visionRobotCenterError)) {
                        // drive forward, slightly turn right
                        // double offset = visionTurnOffset;
                        double error = visionRobotCenterPosition - center;
                        double offset = visionTurnOffset + 
                        3 * visionTurnOffset * (error / (JeVoisInterface.STREAM_WIDTH_PX * JeVoisInterface.VIDEO_SCALE) / 2);
                        drive((visionPower - offset) * speedRate, (visionPower + offset) * speedRate, -1);
                    } else if (center > (visionRobotCenterPosition + visionRobotCenterError)) {
                        // drive forward, sligtly turn left
                        // double offset = visionTurnOffset;
                        double error = center - visionRobotCenterPosition;
                        double offset = visionTurnOffset + 
                        3 * visionTurnOffset * (error / (JeVoisInterface.STREAM_WIDTH_PX * JeVoisInterface.VIDEO_SCALE) / 2);
                        drive((visionPower + offset) * speedRate, (visionPower - offset) * speedRate, -1);
                    } else {
                        // we are right on target, drive forward
                        drive(visionPower * speedRate, visionPower * speedRate, -1);
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
}