import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URL;

/**
 * Created by Camiel on 18-Feb-16.
 */
public class Compass extends JFrame{

    private final JFrame window = new JFrame();
    private int heading = 0;

    String compassRedFile = "images/compassRed.png";
    BufferedImage compassRedImg = null;
    AffineTransform tx = AffineTransform.getRotateInstance(heading, 250, 250);
    AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);


    public Compass() {
        window.setSize(500, 500);
        window.setResizable(false);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        window.setFocusable(true);
        window.setLocationRelativeTo(null);
        window.setTitle("compass");
        window.setVisible(true);


        URL imgUrl = getClass().getClassLoader().getResource(compassRedFile);
        if (imgUrl == null) { System.err.println("Couldn't find file: " + compassRedFile); }
        else {
            try { compassRedImg = ImageIO.read(imgUrl); } catch (IOException ex) { ex.printStackTrace(); }
        }

    }


    public void setHeading(int heading) {
        this.heading = heading;
        paint(window.getGraphics());
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.WHITE);
        g2d.fillRect(0,0,500,500);

        //draw black cross
        g2d.setColor(Color.BLACK);
        g2d.drawLine(250,45,250,470);
        g2d.drawLine(30,250,470,250);
        g2d.drawString("N", 246, 42);
        g2d.drawString("E", 473, 255);
        g2d.drawString("S", 246, 483);
        g2d.drawString("W", 17, 255);

        //draw compass cross
        tx = AffineTransform.getRotateInstance(Math.toRadians (heading), 250, 250); //rotates coordinates around an anchor point
        op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);


        g2d.drawImage(op.filter(compassRedImg, null),1,1,null);
    }
}
