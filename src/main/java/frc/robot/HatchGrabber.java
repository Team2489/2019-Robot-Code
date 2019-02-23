package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class HatchGrabber {

    private DoubleSolenoid hatchGrabber;
    boolean grabbing = false;

    public HatchGrabber(int pcmIndexOne, int pcmIndexTwo){
        hatchGrabber = new DoubleSolenoid(pcmIndexOne, pcmIndexTwo); 
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