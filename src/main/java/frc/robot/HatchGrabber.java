package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class HatchGrabber {

    private DoubleSolenoid hatchGrabber;
    boolean grabbing = false;

    public HatchGrabber(){
        hatchGrabber = new DoubleSolenoid(0,1); 
    }

    public void release() {
        hatchGrabber.set(DoubleSolenoid.Value.kReverse);
        grabbing = false;
    }

    public void grab() {
        hatchGrabber.set(DoubleSolenoid.Value.kForward);
        grabbing = true;
    }

}