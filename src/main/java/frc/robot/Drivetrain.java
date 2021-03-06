package frc.robot;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;

public class Drivetrain {
    private DifferentialDrive ddrive;
    private SpeedControllerGroup left;
    private SpeedControllerGroup right;
    private double kp = 0.01;
    
    private ArduinoI2C aI2C;
    private Spark lightPower;
    
    public double setpoint = 180.0;

    public Drivetrain(int frontLeft, int rearLeft, int frontRight, int rearRight) {
        left = new SpeedControllerGroup(new WPI_TalonSRX(frontLeft), new WPI_TalonSRX(rearLeft));
        right = new SpeedControllerGroup(new WPI_TalonSRX(frontRight), new WPI_TalonSRX(rearRight));
        ddrive = new DifferentialDrive(left, right);
        
        aI2C = new ArduinoI2C();
         
        lightPower = new Spark(0); 
        lightPower.set(1.0);
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

    public String getCenter() {
       return aI2C.read() ;   
    }
}