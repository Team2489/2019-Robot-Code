package frc.robot;

import edu.wpi.first.wpilibj.Servo;

public class VideoTilt {
    private static final double ANGLE_HORIZONTAL = 140;
    private static final double ANGLE_MAX = 170;
    private static final double ANGLE_MIN = 90;
    private static final double ANGLE_INCREMENT = 0.3;

    private Servo servo;
    private double angle;

    public VideoTilt(int pwm) {
        servo = new Servo(pwm);
        setHorizontal();
    }

    public void hold() {
        servo.setAngle(angle);
    }

    public void setHorizontal() {
        angle = ANGLE_HORIZONTAL;
        servo.setAngle(angle);
    }

    public void up() {
        // reverse direction: smaller angle is up
        angle = angle - ANGLE_INCREMENT;
        if (angle < ANGLE_MIN) {
            angle = ANGLE_MIN;
        }
        // System.out.println("up " + angle);
        servo.setAngle(angle);
    }

    public void down() {
        // reverse direction: bigger angle is down;
        angle = angle + ANGLE_INCREMENT;
        if (angle > ANGLE_MAX) {
            angle = ANGLE_MAX;
        }
        // System.out.println("down " + angle);
        servo.setAngle(angle);
    }
}