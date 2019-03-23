package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.*;


public class Arm {

    private TalonSRX motor;
    private double controlParamter = 0.0;
    private double maxAccel = 0.1; // tune this parameter
    private double offset = 0; // tune this parameter like the migos

    private AnalogPotentiometer pot = new AnalogPotentiometer(0, 270, -192);
    
    public Arm(int index) {
        motor = new TalonSRX(index);
        // pot = new AnalogPotentiometer(0, 270, offset);
    }

    public void actuate(double velocity, boolean shouldFreeze) {
        if(!shouldFreeze) {
           motor.set(ControlMode.PercentOutput, -velocity);
        } else {
            motor.set(ControlMode.PercentOutput, limit(controlParamter + 0.2 * velocity));
        }
    }

    private double kP = 0.01, kI = 0.00, kD = 0, kF = 0.27, integral, prev_err;


    public void pidActuate(double setpoint) {
        double err = setpoint - getAngle();
        double deriv = (err - prev_err) / 0.02;
        integral += err * 0.02;
        double controlParamter = kP * err + kI * integral + kD * deriv + kF * Math.cos(setpoint * Math.PI / 180.0);
        System.out.println(controlParamter);
        motor.set(ControlMode.PercentOutput, controlParamter);

    }

    private double limit(double signal) {
        if(signal > 1) return 1;
        if(signal < 1) return -1;
        return signal;
    }

    public double getAngle() {
        return -pot.get() - 155;
    }
}