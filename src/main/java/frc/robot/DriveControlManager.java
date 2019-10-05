package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControlManager {
    private Joystick m_leftStick;
    private Joystick m_rightStick;
    private Joystick xbox;

    public double k = 0.8;
    private double kArm = 0.75;

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
        return m_rightStick.getRawButton(3) || xbox.getRawButton(8);
    }

    public boolean shouldRelease() {
        return m_leftStick.getRawButton(3) || xbox.getRawButton(7);
    }

    public boolean shouldForward() {
        return m_rightStick.getRawButton(5);
    }

    public boolean shouldReverse() {
        return m_leftStick.getRawButton(5);
    }

    public int shouldEnterOrExit() { // 0 = exit, 1 = enter, -1 = human control
        // if(m_rightStick.getTrigger()){
        //     return 0;
        // }else if(m_leftStick.getTrigger()){
        //     return 1;
        // }
        return -1;
    }
    
    public boolean shouldFreezeArm() {
        return false;
    }

    public void updateSquat() {
        // if(m_leftStick)
        // k = (m_leftStick.getZ() + 1) / 2;
        if(m_leftStick.getRawButton(2) && m_rightStick.getRawButton(2)) {
           k = 0.4;
        } else {
           k = 0.8;
        }
        return;
    }

    public boolean shouldVisionDrive() {
        boolean ret = false;
        if(m_rightStick.getRawButton(4)) {
            ret = true;
        }
        return ret;
    }

    public int getVisionHint() {
        int ret = 0;
        // if (m_rightStick.getRawButton(4)) {
        //    ret = 1;
        // } else if(m_leftStick.getRawButton(5)) {
        //    ret = -1;
        // }
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
        if (m_leftStick.getRawButton(10) || xbox.getRawButton(2)) {
            ret = true;
        } 
        return ret;
    }

    public boolean shouldBallDispenserPush() {
        return m_leftStick.getTrigger();
    }

    public boolean shouldBallDispenserRetract() {
        return m_rightStick.getTrigger();
    }

    public boolean shouldTurnLeft() {
        return (m_rightStick.getRawButton(6));
    }

    public boolean shouldTurnRight() {
        return (m_rightStick.getRawButton(11));
    }
}