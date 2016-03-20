import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.Enumeration;

/**
 * Created by Camiel on 21-Feb-16.
 */
public class SerialController implements SerialPortEventListener {

    private static SerialPort serialPort;
    private static final String PORT_NAME = "COM5"; //arduino port name
    private static BufferedReader input; //A BufferedReader which will be fed by a InputStreamReader converting the bytes into characters making the displayed results codepage independent

    private static OutputStream output; // The output stream to the port
    private static final int TIME_OUT = 2000; // Milliseconds to block while waiting for port open
    private static final int DATA_RATE = 115200; // Default bits per second for COM port.

    private static Logger logger = new Logger();
    private static JFrameGUI window = new JFrameGUI();
    private static Metrics metrics = new Metrics();

    public void initialize() {

        CommPortIdentifier portId = null;
        Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

        //First, Find an instance of serial port as set in PORT_NAME.
        while (portEnum.hasMoreElements()) {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            if (currPortId.getName().equals(PORT_NAME)) { portId = currPortId; }
        }
        if (portId == null) { System.out.println("Could not find port: " + PORT_NAME);
            window.setConnectionStatus("Unable to connect with base station on port: " + PORT_NAME);
            return; }

        try {
            // open serial port, and use class name for the appName.
            serialPort = (SerialPort) portId.open(this.getClass().getName(), TIME_OUT);

            // set port parameters
            serialPort.setSerialPortParams(DATA_RATE,
                    SerialPort.DATABITS_8,
                    SerialPort.STOPBITS_1,
                    SerialPort.PARITY_NONE);

            // open the streams
            input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
            output = serialPort.getOutputStream();

            // add event listeners
            serialPort.addEventListener(this);
            serialPort.notifyOnDataAvailable(true);

            // update connection status in GUI
            window.setConnectionStatus("Connected to base station on port: " + PORT_NAME);

        } catch (Exception e) {
            System.err.println(e.toString());
        }
    }


    //close port when done using it to prevent port locking
    public synchronized void close() {
        if (serialPort != null) {
            serialPort.removeEventListener();
            serialPort.close();
        }
    }

    //Handle an event on the serial port. Read the data and print it.
    public synchronized void serialEvent(SerialPortEvent oEvent) {
        if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
            try {

                String message = input.readLine();
                logger.writeLogLine(message);

                if(message != "") {
                    if (message.substring(0,2).equals("|h")) {//check if its a logmessage. logmessage has format '|h0|e0|'
                        window.setConnectionStatus("Connected to base station on port: " + PORT_NAME);
                        String headingString = message.substring(2,message.indexOf("|e"));
                        String elevationString = message.substring((message.indexOf("|e")+2),message.indexOf("|",(message.indexOf("|e")+2)));
                        String yawString = message.substring((message.indexOf("|y")+2),message.indexOf("|",(message.indexOf("|y")+2)));
                        String pitchString = message.substring((message.indexOf("|p")+2),message.indexOf("|",(message.indexOf("|p")+2)));
                        String rollString = message.substring((message.indexOf("|r")+2),message.indexOf("|",(message.indexOf("|r")+2)));
                        String m1String = message.substring((message.indexOf("|m1")+3),message.indexOf("|",(message.indexOf("|m1")+3)));
                        String m2String = message.substring((message.indexOf("|m2")+3),message.indexOf("|",(message.indexOf("|m2")+3)));
                        String m3String = message.substring((message.indexOf("|m3")+3),message.indexOf("|",(message.indexOf("|m3")+3)));
                        String m4String = message.substring((message.indexOf("|m4")+3),message.indexOf("|",(message.indexOf("|m4")+3)));
                        //System.out.println(headingString + "   " + elevationString  + "   " + yawString  + "   " + pitchString  + "   " + rollString);

                        metrics.setHeading(Math.round(Float.parseFloat(headingString)));
                        metrics.setElevation(Math.round(Float.parseFloat(elevationString)));
                        metrics.setYaw(Math.round(Float.parseFloat(yawString)));
                        metrics.setPitch(Math.round(Float.parseFloat(pitchString)));
                        metrics.setRoll(Math.round(Float.parseFloat(rollString)));
                        metrics.setThrottleNW(Math.round(Float.parseFloat(m1String)));
                        metrics.setThrottleNE(Math.round(Float.parseFloat(m2String)));
                        metrics.setThrottleSE(Math.round(Float.parseFloat(m3String)));
                        metrics.setThrottleSW(Math.round(Float.parseFloat(m4String)));

                    }
                }
            } catch (Exception e) {
                System.err.println(e.toString());
                window.setConnectionStatus("Lost connection with base station on port: " + PORT_NAME);
            }
        }
    }


}
