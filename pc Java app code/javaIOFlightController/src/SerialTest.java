import java.io.*;

//Serial library provided by rxtx
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import java.util.Enumeration;


public class SerialTest implements SerialPortEventListener {

    SerialPort serialPort;
    private static final String PORT_NAME = "COM3"; //arduino port name
    private BufferedReader input; //A BufferedReader which will be fed by a InputStreamReader converting the bytes into characters making the displayed results codepage independent
    //private DataInputStream input;

    private OutputStream output; // The output stream to the port
    private static final int TIME_OUT = 2000; // Milliseconds to block while waiting for port open
    private static final int DATA_RATE = 9600; // Default bits per second for COM port.

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
                int inputNextAsciiChar = input.read(); //read raw ascii data from input line
                //String inputline = new DataInputStream().read();

                //read ascii and print
                if (inputNextAsciiChar == 13) {  } //carriage return
                else if (inputNextAsciiChar == 10) { System.out.println(""); } //new line
                else { System.out.print(Character.toString ((char) inputNextAsciiChar)); } //convirt ascii to char and print

            } catch (Exception e) {
                System.err.println(e.toString());
            }
        }
        // Ignore all other eventTypes, TODO: consider the other ones.
    }

    public static void main(String[] args) throws Exception {
        SerialTest main = new SerialTest();
        main.initialize();
        Thread t=new Thread() {
            public void run() {
                //keep this app alive for 1000 seconds,
                try {Thread.sleep(1000000);} catch (InterruptedException ie) {}
            }
        };
        t.start();
        System.out.println("Started");
    }
}
