import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

/**
 * Created by Camiel on 21-Feb-16.
 */
public class Logger {

    private static String logLocation = "C:/Users/Camiel/Documents/Arduino/JavaDroneLogs";
    private static File logFile;
    private static long startTime;

    public Logger() {
        if (logFile == null) {
            try {
                startTime = System.currentTimeMillis();
                String timeLog = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
                logFile = new File(logLocation,(timeLog+".txt"));
                System.out.println("flight log can be found in: " + logFile.getCanonicalPath());
                logFile.createNewFile();
                writeLogLine("Java app started logging.");
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void writeLogLine(String logLine) {
        try {
            String line = getTimestamp() + "-" + logLine;
            BufferedWriter writer = new BufferedWriter(new FileWriter(logFile,true));
            writer.newLine();
            writer.write(line);
            writer.close();

            System.out.println(line);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public String getTimestamp() {
        long timeSinceStart = System.currentTimeMillis() - startTime;
        SimpleDateFormat sdf = new SimpleDateFormat("mm:ss:SSS");
        return sdf.format(new Date(timeSinceStart));
    }



}
