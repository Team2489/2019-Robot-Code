package frc.robot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

//        // Initialize 
//        public static ArduinoI2C aI2C;
//        public static Spark lightPower;

//
//        aI2C = new ArduinoI2C();
// 
//        lightPower = new Spark(0); 
//        lightPower.set(1.0);

//        // Read Value from Pixy2
//        String output = aI2C.read(); 
//        SmartDashboard.putString("aI2C", output);

public class ArduinoI2C extends SendableBase {
		private static I2C Wire = new I2C(Port.kOnboard, 4);//uses the i2c port on the RoboRIO
															//uses address 4, must match arduino
		private static final int MAX_BYTES = 32;
		
		public ArduinoI2C() {
			setName("ArduinoI2C");
		}
		
		public void write(String input){//writes to the arduino 
				char[] CharArray = input.toCharArray();//creates a char array from the input string
				byte[] WriteData = new byte[CharArray.length];//creates a byte array from the char array
				for (int i = 0; i < CharArray.length; i++) {//writes each byte to the arduino
					WriteData[i] = (byte) CharArray[i];//adds the char elements to the byte array 
				}
				Wire.transaction(WriteData, WriteData.length, null, 0);//sends each byte to arduino
		}
		
		public String read(){//function to read the data from arduino
			byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
			Wire.read(4, MAX_BYTES, data);//use address 4 on i2c and store it in data
			String output = new String(data);//create a string from the byte array
			int pt = output.indexOf((char)255);
			return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
																   //sorry :(
		}
		
		@Override
		public void initSendable(SendableBuilder builder) {
		    NetworkTableEntry entryCenter = builder.getEntry("Center");
		    builder.setUpdateTable(() -> {
		      String data = read();
		      entryCenter.setString(data);
		    });
		}
}
