package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
public class HatchGrabber {

    private DoubleSolenoid hatchGrabber;
    boolean grabbing = false;
    private Compressor pneumaticsCompressor;

    public HatchGrabber(int pcmIndexOne, int pcmIndexTwo){
        hatchGrabber = new DoubleSolenoid(20, pcmIndexOne, pcmIndexTwo); 
        pneumaticsCompressor = new Compressor(20);
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