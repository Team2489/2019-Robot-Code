package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControlManager {
    private Joystick m_leftStick;
    private Joystick m_rightStick;
    private Joystick xbox;

    private double k = 0.8;
    private double kArm = 0.75;
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
        return m_rightStick.getRawButton(2) || m_rightStick.getRawButton(3) || xbox.getRawButton(8);
    }

    public boolean shouldRelease() {
        return m_leftStick.getRawButton(2) || m_leftStick.getRawButton(3) || xbox.getRawButton(7);
    }

    public int shouldEnterOrExit() { // 0 = exit, 1 = enter, -1 = human control
        if(m_rightStick.getTrigger()){
            return 0;
        }else if(m_leftStick.getTrigger()){
            return 1;
        }
        return -1;
    }
    
    public boolean shouldFreezeArm() {
        return false;
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

    public boolean shouldVisionDrive() {
        boolean ret = false;
        if(m_rightStick.getRawButton(4) || m_leftStick.getRawButton(5)) {
            ret = true;
        }
        return ret;
    }

    public int getVisionHint() {
        int ret = 0;
        if (m_rightStick.getRawButton(4)) {
            ret = 1;
        } else if(m_leftStick.getRawButton(5)) {
            ret = -1;
        }
        return ret;
    }

    public boolean shouldVtiltUp() {
        boolean ret = false;
        if (m_rightStick.getRawButton(9)) {
            ret = true;
        } 
        // System.out.println("shouldVtilitUp() = " + ret);
        return ret;
    }

    public boolean shouldVtiltDown() {
        boolean ret = false;
        if (m_rightStick.getRawButton(10)) {
            ret = true;
        } 
        // System.out.println("shouldVtiltDown() = " + ret);
        return ret;
    }

    public boolean shouldVtiltHorizontal() {
        boolean ret = false;
        if (m_leftStick.getRawButton(10)) {
            ret = true;
        } 
        return ret;
    }

    public boolean shouldJevoisVisionMode() {
        boolean ret = false;
        if (m_rightStick.getRawButton(6)) {
            ret = true;
        }
        return ret;
    }

    public boolean shouldJevoisHumanMode() {
        boolean ret = false;
        if (m_rightStick.getRawButton(7)) {
            ret = true;
        }
        return ret;
    }
}