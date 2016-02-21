import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URL;
import java.nio.Buffer;

/**
 * Created by Camiel on 21-Feb-16.
 */
public class JFrameGUI extends JPanel {

    private final JFrame window;
    private static Metrics metrics = new Metrics();

    BufferedImage compassImg = null;
    BufferedImage backgroundImg = null;
    BufferedImage airspeedImg = null;
    BufferedImage elevationImg = null;
    BufferedImage headingImg = null;
    BufferedImage pointDImg = null;
    BufferedImage pointLImg = null;
    BufferedImage pointRImg = null;
    AffineTransform tx = AffineTransform.getRotateInstance(metrics.getHeading(), 150, 150);
    AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);


    public JFrameGUI() {
        compassImg = loadImg("images/compass.png");
        backgroundImg = loadImg("images/GUI.png");
        airspeedImg = loadImg("images/airspeed.png");
        elevationImg = loadImg("images/elevation.png");
        headingImg = loadImg("images/heading.png");
        pointDImg = loadImg("images/pointD.png");
        pointLImg = loadImg("images/pointL.png");
        pointRImg = loadImg("images/pointR.png");

        window = new JFrame();
        window.setSize(1200, 1000);
        window.setResizable(false);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        window.setFocusable(true);
        window.setLocationRelativeTo(null);
        window.setTitle("Drone UI");
        window.setVisible(true);

        window.add(this);
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        //draw yaw circle
        tx = AffineTransform.getRotateInstance(Math.toRadians (360 - metrics.getHeading()), 250, 250); //rotates coordinates around an anchor point
        op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
        g2d.drawImage(op.filter(headingImg, null),587,387,null);
        g2d.drawImage(pointDImg,823,374,null);

        //draw main background
        g2d.drawImage(backgroundImg,0,0,null);

        //draw compass
        tx = AffineTransform.getRotateInstance(Math.toRadians (360 - metrics.getHeading()), 150, 150); //rotates coordinates around an anchor point
        op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
        g2d.drawImage(op.filter(compassImg, null),100,100,null);

        //draw throttle levels
        g2d.setColor(Color.decode("#C72A34"));
        int throttleNW = (int) metrics.getThrottleNW(); g2d.fillRect(52,729-(2*throttleNW),50,2*throttleNW); g2d.drawString(Integer.toString(throttleNW) + "%", 146, 657);
        int throttleNE = (int) metrics.getThrottleNE(); g2d.fillRect(402,729-(2*throttleNE),50,2*throttleNE); g2d.drawString(Integer.toString(throttleNE) + "%", 340, 657);
        int throttleSE = (int) metrics.getThrottleSE(); g2d.fillRect(402,949-(2*throttleSE),50,2*throttleSE); g2d.drawString(Integer.toString(throttleSE) + "%", 340, 857);
        int throttleSW = (int) metrics.getThrottleSW(); g2d.fillRect(52,949-(2*throttleSW),50,2*throttleSW); g2d.drawString(Integer.toString(throttleSW) + "%", 146, 857);

        //draw airspeed meter
        float airspeed = metrics.getAirspeed(); // 0 x1:
        g2d.drawImage(airspeedImg, 530, 100, 580, 400, 20, 2499, 70, 2800, null);
        g2d.drawImage(pointLImg,565,233,null);

        //draw altidude meter
        float elevation = metrics.getElevation(); // 0 x1:
        g2d.drawImage(elevationImg, 1085, 100, 1135, 400, 0, 2499, 50, 2800, null);
        g2d.drawImage(pointRImg,1068,234,null);

        repaint();
    }


    private BufferedImage loadImg(String filename) {
        BufferedImage img = null;
        URL imgUrl = getClass().getClassLoader().getResource(filename);
        if (imgUrl == null) { System.err.println("Couldn't find file: " + filename); }
        else {
            try { img = ImageIO.read(imgUrl); } catch (IOException ex) { ex.printStackTrace(); }
        }
        return img;
    }
}
