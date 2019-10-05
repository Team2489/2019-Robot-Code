package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class HatchGrabber {

    private DoubleSolenoid hatchGrabber;
    // private Compressor pneumaticsCompressor;
    boolean grabbing = false;

    public HatchGrabber(int pcmIndexOne, int pcmIndexTwo){
        // pneumaticsCompressor = new Compressor(20);
        hatchGrabber = new DoubleSolenoid(20, pcmIndexOne, pcmIndexTwo); 
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