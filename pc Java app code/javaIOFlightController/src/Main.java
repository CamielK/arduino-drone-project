import java.io.*;

//Serial library provided by rxtx
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import java.util.Enumeration;


public class Main {

    public static void main(String[] args) throws Exception {
        SerialController controller = new SerialController();
        controller.initialize();

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
