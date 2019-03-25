package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Sonar extends SendableBase {
    public AnalogInput  m_analog;

	public Sonar (int channel) {
		m_analog = new AnalogInput(channel);
		// m_analog.setOversampleBits(5);
		m_analog.setAverageBits(10);
		setName("MaxbotixSonar", m_analog.getChannel());
	}
	public double getDistance () {
		double ret = getDistanceCm();
		ret = ret / 2.54;
		return ret;
	}
	public double getDistanceCm() {
		double ret = 0;
		ret = m_analog.getVoltage() / RobotController.getVoltage5V();
		// ret = m_analog.getVoltage() / 5.0;
		// arduino way
		ret = (ret * 1024) / 2;
		return ret;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
	    NetworkTableEntry entryDistance = builder.getEntry("Distance Inches");
	    builder.setUpdateTable(() -> {
	      double data = getDistance();
	      entryDistance.setDouble(data);
	    });
	}
}
