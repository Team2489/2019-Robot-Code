/*
 * Copied and modified from:
 * https://github.com/RobotCasserole1736/JeVoisTester/blob/master/JeVoisTest/src/org/usfirst/frc/team1736/robot/JeVoisInterface.java
 */

package frc.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

import frc.robot.VisionTarget;

public class JeVoisInterface {
    
    // Serial Port Constants 
    private static final int BAUD_RATE = 115200;
    
    // MJPG Streaming Constants 
    private static final int MJPG_STREAM_PORT = 1180;
    
    // Packet format constants 
    private static final String PACKET_START_CHAR = "{";
    private static final String PACKET_END_CHAR = "}";
    private static final String PACKET_DILEM_CHAR = ";";
    private static final String TARGET_DILEM_CHAR = ",";

    private static final int TARGET_FIELDS_NUM = 5;

    private static final int debug = 0;
    
    // Confgure the camera to stream debug images or not.
    private boolean broadcastUSBCam = false;
    
    // When not streaming, use this mapping
    private static final int NO_STREAM_MAPPING = 0;
    
    // When streaming, use this set of configuration
    // high resolution
    // public static final int STREAM_WIDTH_PX = 640;
    // public static final int STREAM_HEIGHT_PX = 480;
    // public static final int STREAM_RATE_FPS = 15;

    // low resolution used as base VIDEO_SCALE=1
    public static final int STREAM_WIDTH_PX = 320;
    public static final int STREAM_HEIGHT_PX = 240;
    public static final int STREAM_RATE_FPS = 15;
    
    public static final int VIDEO_SCALE = 2;

    private static final int THREAD_SLEEP_INTERVAL = 50; //ms
    
    // Serial port used for getting target data from JeVois 
    private SerialPort visionPort = null;
    
    // USBCam and server used for broadcasting a webstream of what is seen 
    private UsbCamera visionCam = null;
    // private MjpegServer camServer = null;
    
    // Status variables 
    private boolean dataStreamRunning = false;
    private boolean camStreamRunning = false;
    private boolean visionOnline = false;

    // Packet rate performace tracking
    private double packetRxTime = 0;
    private double prevPacketRxTime = 0;
    private double packetRate_PPS = 0;

    // Info about the JeVois performace & status
    private double jeVoisCpuTempC = 0;
    private double jeVoisCpuLoadPct = 0;
    private double jeVoisFramerateFPS = 0;
    private double packetRxRatePPS = 0;
    
    private ArrayList<VisionTarget> visionTargets = null;

    private boolean currentIsVisionMode = true;
    private boolean humanModeRequested = false;
    private boolean visionModeRequested = false;

    //=======================================================
    //== BEGIN PUBLIC INTERFACE
    //=======================================================

    /**
     * Constructor (simple). Opens a USB serial port to the JeVois camera, sends a few test commands checking for error,
     * then fires up the user's program and begins listening for target info packets in the background
     */
    public JeVoisInterface() {
        this(false); //Default - stream disabled, just run serial.
    }

    /**
     * Constructor (more complex). Opens a USB serial port to the JeVois camera, sends a few test commands checking for error,
     * then fires up the user's program and begins listening for target info packets in the background.
     * Pass TRUE to additionaly enable a USB camera stream of what the vision camera is seeing.
     */
    public JeVoisInterface(boolean useUSBStream) {
        int retry_counter = 0;
        
        //Retry strategy to get this serial port open.
        //I have yet to see a single retry used assuming the camera is plugged in
        // but you never know.
        while(visionPort == null && retry_counter++ < 10){
            try {
                if (debug >= 1) {
                    System.out.print("Creating JeVois SerialPort...");
                }
                visionPort = new SerialPort(BAUD_RATE,SerialPort.Port.kUSB);
                if (debug >= 1) {
                    System.out.println("SUCCESS!!");
                }
            } catch (Exception e) {
                if (debug >= 1) {
                    System.out.println("FAILED!!");
                }
                e.printStackTrace();
                sleep(500);
                if (debug >= 1) {               
                    System.out.println("Retry " + Integer.toString(retry_counter));
                }
            }
        }
      
        //Report an error if we didn't get to open the serial port
        if(visionPort == null){
            driverStationReportError("Cannot open serial port to JeVois. Not starting vision system.", false);
            return;
        }
        
        //Test to make sure we are actually talking to the JeVois
        if(sendPing() != 0){
            driverStationReportError("JeVois ping test failed. Not starting vision system.", false);
            return;
        }
        
        //Ensure the JeVois is starting with the stream off.
        stopDataOnlyStream();

        setCameraStreamActive(useUSBStream);
        start();

        //Start listening for packets
        packetListenerThread.setDaemon(true);
        packetListenerThread.start();

    } 

    public void start(){
        if(broadcastUSBCam){
            //Start streaming the JeVois via webcam
            //This auto-starts the serial stream
            startCameraStream(); 
        } else {
            startDataOnlyStream();
        }
    }

    public void stop(){
        if(broadcastUSBCam){
            //Start streaming the JeVois via webcam
            //This auto-starts the serial stream
            stopCameraStream(); 
        } else {
            stopDataOnlyStream();
        }
    }
    
    /**
     * Send commands to the JeVois to configure it for image-processing friendly parameters
     */
    public void setCamVisionProcMode() {
        if (visionPort != null){
            sendCmdAndCheck("setcam autoexp 1"); //Disable auto exposure
            sendCmdAndCheck("setcam absexp 25"); //Force exposure to a low value for vision processing
        }
    }
    
    public synchronized void setCamVisionProcModeAsync() {
        visionModeRequested = true;
    }

    /**
     * Send parameters to the camera to configure it for a human-readable image
     */
    public void setCamHumanDriverMode() {
        if (visionPort != null){
            sendCmdAndCheck("setcam autoexp 0"); //Enable AutoExposure
        }
    }

    public synchronized void setCamHumanDriverModeAsync() {
        humanModeRequested = true;
    }

    /*
     * Main getters/setters
     */

    /**
     * Set to true to enable the camera stream, or set to false to stream serial-packets only.
     * Note this cannot be changed at runtime due to jevois constraints. You must stop whatatever processing
     * is going on first.
     */
    public void setCameraStreamActive(boolean active){
        if(dataStreamRunning == false){
            broadcastUSBCam = active;
        } else {
            driverStationReportError("Attempt to change cal stream mode while JeVois is still running. This is disallowed.", false);
        }
        

    }
        
    /**
     * Returns true when the roboRIO is recieving packets from the JeVois, false if no packets have been recieved.
     * Other modules should not use the vision processing results if this returns false.
     */
    public boolean isVisionOnline() {
        return visionOnline;
    }
        
    /**
     * Returns the JeVois's most recently reported CPU Temperature in deg C
     */
    public double getJeVoisCPUTemp_C(){
        return jeVoisCpuTempC;
    }

    /**
     * Returns the JeVois's most recently reported CPU Load in percent of max
     */
    public double getJeVoisCpuLoad_pct(){
        return jeVoisCpuLoadPct;
    }

    /**
     * Returns the JeVois's most recently reported pipline framerate in Frames per second
     */
    public double getJeVoisFramerate_FPS(){
        return jeVoisFramerateFPS;
    }

    /**
     * Returns the roboRIO measured serial packet recieve rate in packets per second
     */
    public int getPacketRxRate_PPS(){
    	if(visionOnline){
    		return (int)Math.round(packetRxRatePPS);
    	} else {
    		return 0;
    	}
    }

    //=======================================================
    //== END PUBLIC INTERFACE
    //=======================================================

    
    /**
     * This is the main perodic update function for the Listener. It is intended
     * to be run in a background task, as it will block until it gets packets. 
     */
    private void backgroundUpdate(){
        
        // Grab packets and parse them.
        String packet;
        
        prevPacketRxTime = packetRxTime;
        packet = blockAndGetPacket(2.0);
        
        if(packet != null){
            packetRxTime = timerGetFPGATimestamp();
            if( parsePacket(packet, packetRxTime) == 0){
                visionOnline = true;
                packetRxRatePPS = 1.0/(packetRxTime - prevPacketRxTime);
            } else {
                visionOnline = false;
            }
            
        } else {
            visionOnline = false;
            driverStationReportWarning("Cannot get packet from JeVois Vision Processor", false);
        }
        
    }

    /**
     * Send the ping command to the JeVois to verify it is connected
     * @return 0 on success, -1 on unexpected response, -2 on timeout
     */
    private int sendPing() {
        int retval = -1;
        if (visionPort != null){
            String response = sendCmdAndCheck("ping");
            if (response == "-1" ) {
                retval = -1;
            } else if (response == "-2") {
                retval = -2;
            } else {
                retval = 0;
            }
        }
        return retval;
    }

    private void startDataOnlyStream(){
        //Send serial commands to start the streaming of target info
        sendCmdAndCheck("setmapping " + Integer.toString(NO_STREAM_MAPPING));
        sendCmdAndCheck("streamon");
        dataStreamRunning = true;
    }

    private void stopDataOnlyStream(){
        //Send serial commands to stop the streaming of target info
        sendCmdAndCheck("streamoff");
        dataStreamRunning = false;
    }
    

    /**
     * Open an Mjpeg streamer from the JeVois camera
     */
    private void startCameraStream(){
        try{
            if (debug >= 1) {
                System.out.print("Starting JeVois Cam Stream...");
            }
            // visionCam = CameraServer.getInstance().startAutomaticCapture("JeVois Camera", 1);
            visionCam = CameraServer.getInstance().startAutomaticCapture(1);
            // two possible formats PixelFormat.kBGR and PixelFormat.kMJPEG
            visionCam.setVideoMode(PixelFormat.kMJPEG ,
                                   STREAM_WIDTH_PX * VIDEO_SCALE,
                                   STREAM_HEIGHT_PX * VIDEO_SCALE,
                                   STREAM_RATE_FPS);

            // visionCam = new UsbCamera("VisionProcCam", 0);
            // visionCam.setVideoMode(PixelFormat.kBGR , STREAM_WIDTH_PX, STREAM_HEIGHT_PX, STREAM_RATE_FPS);
            // camServer = new MjpegServer("VisionCamServer", MJPG_STREAM_PORT);
            // camServer.setSource(visionCam);

            camStreamRunning = true;
            dataStreamRunning = true;
            if (debug >= 1) {
                System.out.println("SUCCESS!!");
            }
        } catch (Exception e) {
            driverStationReportError("Cannot start camera stream from JeVois", false);
            e.printStackTrace();
        }
    }
    
    /**
     * Cease the operation of the camera stream. Unknown if needed.
     */
    private void stopCameraStream(){
        if(camStreamRunning){
            // camServer.free();
            visionCam.free();
            camStreamRunning = false;
            dataStreamRunning = false;
        }
    }
    
    /**
     * Sends a command over serial to JeVois and returns immediately.
     * @param cmd String of the command to send (ex: "ping")
     * @return number of bytes written
     */
    private int sendCmd(String cmd){
        int bytes;
        bytes = visionPort.writeString(cmd + "\n");
        if (debug >= 10) {
            System.out.println("wrote " +  bytes + "/" + (cmd.length()+1) + " bytes, cmd: " + cmd);
        }
        return bytes;
    };
    
    /**
     * Sends a command over serial to the JeVois, waits for a response, and checks that response
     * Automatically ends the line termination character.
     * @param cmd String of the command to send (ex: "ping")
     * @return 0 if OK detected, -1 if ERR detected, -2 if timeout waiting for response
     */
    public String sendCmdAndCheck(String cmd){
        String retval;
        drain();
        sendCmd(cmd);
        retval = blockAndCheckForOK(1.0);
        if(retval == "-1"){
            if (debug >= 10) {
                System.out.println(cmd + " Produced an error");
            }
        } else if (retval == "-2") {
            if (debug >= 10) {
                System.out.println(cmd + " timed out");
            }
        }
        if (debug >= 100) {
            System.out.println(cmd + " got response: " + retval);
        }
        return retval;
    };

    private void drain() {
        if (visionPort != null) {
            while (visionPort.getBytesReceived() > 0) {
                String rxString = visionPort.readString();
            }
        }
    }

    //Persistent but "local" variables for getBytesPeriodic()
    private String getBytesWork = "";
    private int loopCount = 0;
    /**
     * Read bytes from the serial port in a non-blocking fashion
     * Will return the whole thing once the first "OK" or "ERR" is seen in the stream.
     * Returns null if no string read back yet.
     */
    private String getCmdResponseNonBlock() {
        String retval =  null;
        if (visionPort != null){
            if (visionPort.getBytesReceived() > 0) {
                String rxString = visionPort.readString();
                if (debug >= 1) {
                    System.out.println("Waited: " + loopCount + " loops, Rcv'd: " + rxString);
                }
                getBytesWork += rxString;
                if(getBytesWork.contains("OK") || getBytesWork.contains("ERR")){
                    retval = getBytesWork;
                    getBytesWork = "";
                    if (debug >= 1) {
                        System.out.println(retval);
                    }
                }
                loopCount = 0;
            } else {
                ++loopCount;
            }
        }
        return retval;
    }
    
    /** 
     * Blocks thread execution till we get a response from the serial line
     * or timeout. 
     * Return values:
     *  response = camera said in response
     * -1 = ERR in response
     * -2 = No token found before timeout_s
     */
    private String blockAndCheckForOK(double timeout_s){
        String retval = "-2";
        double startTime = timerGetFPGATimestamp();
        String testStr = "";
        if (visionPort != null){
            while(timerGetFPGATimestamp() - startTime < timeout_s){
                if (visionPort.getBytesReceived() > 0) {
                    testStr += visionPort.readString();
                    if(testStr.contains("OK")){
                        retval = testStr;
                        break;
                    }else if(testStr.contains("ERR")){
                    	driverStationReportError("JeVois reported error:\n" + testStr, false);
                        retval = "-1";
                        break;
                    }

                } else {
                    sleep(THREAD_SLEEP_INTERVAL);
                }
            }
        }
        return retval;
    }
    
    /** 
     * Blocks thread execution till we get a valid packet from the serial line
     * or timeout. 
     * Return values:
     *  String = the packet 
     *  null = No full packet found before timeout_s
     */
    private String blockAndGetPacket(double timeout_s){
        String retval = null;
        int endIdx = -1;
        int startIdx = -1;

        drain();
        sendCmd("t");
        String response = blockAndCheckForOK(timeout_s);
        if (response == "-1") {
            if (debug >= 10) {
                System.out.println("blockAndGetPacket: got error");
            }
        } else if (response == "-2") {
            if (debug >= 10) {
                System.out.println("blockAndGetPacket: timeout");
            }
        } else {
            if (debug >= 100) {
                System.out.println("blockAndGetPacket: got response: " + response);
            }
            
            // does response contains end of packet char
            endIdx = response.lastIndexOf(PACKET_END_CHAR);
            if (endIdx != -1) {
                // find start of packet char
                startIdx = response.lastIndexOf(PACKET_START_CHAR, endIdx);
                if (startIdx != -1) {
                    retval = response.substring(startIdx+1, endIdx);
                }
            }
        }
        return retval;
    }
    
    /**
     * Private wrapper around the Thread.sleep method, to catch that interrupted error.
     * @param time_ms
     */
    private void sleep(int time_ms){
        try {
            Thread.sleep(time_ms);
        } catch (InterruptedException e) {
            if (debug >= 1) {
                System.out.println("DO NOT WAKE THE SLEEPY BEAST");
            }
            e.printStackTrace();
        }
    }
    
    /**
     * Mostly for debugging. Blocks execution forever and just prints all serial 
     * characters to the console. It might print a different message too if nothing
     * comes in.
     */
    public void blockAndPrintAllSerial(){
        if (visionPort != null){
            while(!Thread.interrupted()){
                if (visionPort.getBytesReceived() > 0) {
                    System.out.print(visionPort.readString());
                } else {
                    System.out.println("Nothing Rx'ed");
                    sleep(100);
                }
            }
        }

    }
    
    /**
     * Parse individual targets from a packet
     * @param pkt
     */
    public int parsePacket(String pkt, double rx_Time){
        int retval = -1;

        ArrayList<VisionTarget> currentVisionTargets = null;

        if (debug >= 10) {
            System.out.println("parsePacket: got packet: " + pkt);
        }
       
        if (pkt == "") {
            // No targets
        } else {
            String[] targets = pkt.split(PACKET_DILEM_CHAR);

            currentVisionTargets = new ArrayList<VisionTarget>();
            
            for (String target : targets) {
                if (debug >= 100) {
                    System.out.println("parsePacket: got target: " + target);
                }

                String[] tokens = target.split(",");
                if (debug >= 1000) {
                    System.out.println("parsePacket: tokens.length: " + tokens.length);
                }
                if (tokens.length == TARGET_FIELDS_NUM) {
                    String id = tokens[0];
                    try {
                        int x = Integer.parseInt(tokens[1]);
                        int y = Integer.parseInt(tokens[2]);
                        int height = Integer.parseInt(tokens[3]);
                        int width = Integer.parseInt(tokens[4]);

                        VisionTarget vt = new VisionTarget(id, x, y, height, width);
                        currentVisionTargets.add(vt);
                        retval = 0;
                    } catch(NumberFormatException ex) {
                        retval = -2;
                    }
                }
            }
        }

        if (retval == 0) {
            synchronized(this) {
                visionTargets = currentVisionTargets;
            }
        } else {
          if (debug >= 10) {
              System.out.println("parsePacket: got error: " + retval);
          }
        }
          
        
        return retval;        
    }

    public synchronized ArrayList<VisionTarget> getVisionTargets() {
        return visionTargets;
    }

    private void driverStationReportError(String error, boolean printTrace) {
        DriverStation.reportError(error, printTrace);
        // System.out.println("DriverStation ERROR:" + error);
    }

    private void driverStationReportWarning(java.lang.String error, boolean printTrace) {
        DriverStation.reportWarning(error, printTrace);
        // System.out.println("DriverStation WARNING:" + error);
    }

    private double timerGetFPGATimestamp() {
        // return Timer.getFPGATimestamp();
        return System.currentTimeMillis() / 1000;
    }
    
    /**
     * This thread runs a periodic task in the background to listen for vision camera packets.
     */
    Thread packetListenerThread = new Thread(new Runnable(){
        public void run(){
            try {
                while(!Thread.interrupted()){
                    boolean doVisionMode = true;
                    boolean doSwitchToVision = false;
                    boolean doSwitchToHuman = false;

                    synchronized(this) {
                        if (humanModeRequested) {
                            if (currentIsVisionMode) {
                                doSwitchToHuman = true;
                                currentIsVisionMode = false;
                            }
                        }
                        if (visionModeRequested) {
                            if (!currentIsVisionMode) {
                                doSwitchToVision = true;
                                currentIsVisionMode = true;
                            }
                        }
                        humanModeRequested = false;
                        visionModeRequested = false;

                        doVisionMode = currentIsVisionMode;
                    }

                    if (doSwitchToHuman) {
                        setCamHumanDriverMode();
                    }
                    if (doSwitchToVision) {
                        setCamVisionProcMode();
                    }
                    if (doVisionMode) {
        		        backgroundUpdate();
                        // sleep for 20ms, i.e update with 50 fps
                        sleep(THREAD_SLEEP_INTERVAL);
                    } else {
                        // Human Output on Camera check if status changes
                        sleep(THREAD_SLEEP_INTERVAL * 5);
                    }
                }
            } catch (Exception ex) {
                synchronized(this) {
                    visionTargets = null;
                }
            }
        }
    });
}
