import java.io.*;

//Serial library provided by rxtx
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import java.util.Enumeration;


public class Main implements SerialPortEventListener {

    SerialPort serialPort;
    private static final String PORT_NAME = "COM3"; //arduino port name
    private BufferedReader input; //A BufferedReader which will be fed by a InputStreamReader converting the bytes into characters making the displayed results codepage independent

    private OutputStream output; // The output stream to the port
    private static final int TIME_OUT = 2000; // Milliseconds to block while waiting for port open
    private static final int DATA_RATE = 115200; // Default bits per second for COM port.

    private static Logger logger = new Logger();
    private static JFrameGUI window = new JFrameGUI();
    private static Metrics metrics = new Metrics();
    private int heading = 0;

    public void initialize() {

        CommPortIdentifier portId = null;
        Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

        //First, Find an instance of serial port as set in PORT_NAME.
        while (portEnum.hasMoreElements()) {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            if (currPortId.getName().equals(PORT_NAME)) { portId = currPortId; }
        }
        if (portId == null) { System.out.println("Could not find port: " + PORT_NAME); return; }

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
                        String headingString = message.substring(2,message.indexOf("|e"));
                        String elevationString = message.substring((message.indexOf("|e")+2),message.indexOf("|",(message.indexOf("|e")+2)));
                        //System.out.println(headingString + "   " + elevationString);

                        heading = Math.round(Float.parseFloat(headingString));
                        metrics.setHeading(heading);


                        metrics.setThrottleNW(20);metrics.setThrottleNE(30);metrics.setThrottleSW(10);metrics.setThrottleSE(25);

                    }
                }
            } catch (Exception e) {
                System.err.println(e.toString());
            }
        }
    }

    public static void main(String[] args) throws Exception {
        Main main = new Main();
        main.initialize();
        Thread t = new Thread() {
            public void run() {
                boolean running = true;
                long lastTime = System.nanoTime(); //save start time in nanoseconds since epoch
                double nsPerTick = 1000000000D/20D; //nano seconds per tick (60TPS)
                double delta = 0;

                while (running) {

                    long now = System.nanoTime(); //save current time in nanoseconds
                    delta += (now-lastTime)/nsPerTick; //get difference between start time and current time. devide by nsPerTick to get the passed ammount of a tick. this is added to delta
                    lastTime = now; //update last run time to current run time

                    while (delta >= 1) { //when delta becomes greater than 1, a tick has passed
                        delta -= 1;
                    }

                }
            }
        };
        t.start();
        System.out.println("Java app started");

    }
}
