package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControlManager {
    private Joystick m_leftStick;
    private Joystick m_rightStick;
    private Joystick xbox;

    private double k = 0.75;
    private double kArm = 0.7;
    private boolean squat = false;
    
    public DriveControlManager() {
        m_leftStick = new Joystick(0);
        m_rightStick = new Joystick(1);
        xbox = new Joystick(2);
    }

    public double getLeftVelocity() {
        return -k * m_leftStick.getY();
    }

    public double getRightVelocity() {
        return -k * m_rightStick.getY();
    }

    public double getArmVelocity() {
        return xbox.getY() * kArm;
    }

    public boolean shouldGrab() {
        return m_rightStick.getRawButton(2);
    }

    public boolean shouldRelease() {
        return m_leftStick.getRawButton(2);
    }

    public int shouldEnterOrExit() {
        if(m_rightStick.getTrigger()){
            return 0;
        }else if(m_leftStick.getTrigger()){
            return 1;
        }
        return -1;
    }
    
    public boolean shouldFreezeArm() {
        return xbox.getRawButton(8);
    }

    public void updateSquat() {
        if(m_leftStick.getRawButton(4) && m_rightStick.getRawButton(5)) {
            squat = true;
            k = 0.6;
        } else {
            squat = false;
            k = 0.75;
        }
    }
}