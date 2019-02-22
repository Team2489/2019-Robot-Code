package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Arm {

    private TalonSRX motor;
    private double controlParamter = 0.0;
    private double maxAccel = 0.1; // tune this parameter

    public Arm(int index) {
        motor = new TalonSRX(index);
    }

    public void actuate(double velocity, boolean shouldFreeze) {
        if(!shouldFreeze) {
            double delta = velocity - controlParamter;
            if(delta < 0)
                controlParamter += Math.max(delta, -maxAccel);
            if(delta > 0)
                controlParamter += Math.min(delta, maxAccel);
            motor.set(ControlMode.PercentOutput, controlParamter);
        } else {
            motor.set(ControlMode.PercentOutput, limit(controlParamter + 0.2 * velocity));
        }
    }

    private double limit(double signal) {
        if(signal > 1) return 1;
        if(signal < 1) return -1;
        return signal;
    }
}