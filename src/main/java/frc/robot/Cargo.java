package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class Cargo {

    private DoubleSolenoid cargoReleaser;

    public Cargo(int pcmIndexOne, int pcmIndexTwo){
        cargoReleaser = new DoubleSolenoid(pcmIndexOne, pcmIndexTwo); 
    }

    public void releaseBall() {
        cargoReleaser.set(DoubleSolenoid.Value.kReverse);
    }

    public void getBall() {
        cargoReleaser.set(DoubleSolenoid.Value.kForward);
    }

}