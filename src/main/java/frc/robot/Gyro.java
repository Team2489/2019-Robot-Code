package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;


public class Gyro {

    AHRS ahrs;
    public Gyro() {
        try {

            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      
            ahrs = new AHRS(SPI.Port.kMXP); 
      
        } catch (RuntimeException ex ) {
      
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      
        }
    }

    public double getGyroAngle() {
        return (ahrs.getAngle() + 360.0) % 360.0;
    }

    public double angleDifference(double target) {
    return (target - getGyroAngle() + 540) % 360 - 180;
    }

}