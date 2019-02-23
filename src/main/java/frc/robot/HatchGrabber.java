package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class HatchGrabber {

    private DoubleSolenoid hatchGrabber;

    public HatchGrabber(int index){
        hatchGrabber = new DoubleSolenoid(0,1); 
    }

    public void release() {
        hatchGrabber.set(DoubleSolenoid.Value.kReverse);
    }

    public void grab() {
        hatchGrabber.set(DoubleSolenoid.Value.kForward);
    }

}