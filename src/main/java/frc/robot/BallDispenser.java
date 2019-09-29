package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
public class BallDispenser {

    private DoubleSolenoid ballDispenser;
    boolean extended = false;

    public BallDispenser(int pcmIndexOne, int pcmIndexTwo){
        ballDispenser = new DoubleSolenoid(20, pcmIndexOne, pcmIndexTwo); 
    }


    public void push() {
        ballDispenser.set(DoubleSolenoid.Value.kReverse);
        extended = true;
    }

    public void retract() {
        ballDispenser.set(DoubleSolenoid.Value.kForward);
        extended = false;
    }

}