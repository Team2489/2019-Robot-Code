package frc.robot;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain {
    private DifferentialDrive ddrive;
    private SpeedControllerGroup left;
    private SpeedControllerGroup right;

    private Timer exitTimer;

    public boolean underDriverControl = true;

    public Drivetrain(int frontLeft, int rearLeft, int frontRight, int rearRight) {
        left = new SpeedControllerGroup(new WPI_TalonSRX(frontLeft), new WPI_TalonSRX(rearLeft));
        right = new SpeedControllerGroup(new WPI_TalonSRX(frontRight), new WPI_TalonSRX(rearRight));
        ddrive = new DifferentialDrive(left, right);
        
        exitTimer = new Timer();
    }

    public void drive(double leftVelocity, double rightVelocity, boolean exit) {
        if(!exit) {
            ddrive.tankDrive(leftVelocity, rightVelocity);
            underDriverControl = true;
        }
        else {
            if(underDriverControl) {
                exitTimer.start();
                underDriverControl = false;
            }

            if(exitTimer.get() >= 2) {
                underDriverControl = true;
                exitTimer.reset();
            } else {
                underDriverControl = false;
            }

            if(!underDriverControl) {
                ddrive.tankDrive(-0.6, -0.6);
            }
        }
    }
    

    
}