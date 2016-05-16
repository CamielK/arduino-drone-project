import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URL;

/**
 * Created by Camiel on 21-Feb-16.
 */
public class JFrameGUI extends JPanel implements ActionListener {

    private final JFrame window;
    private static Metrics metrics = new Metrics();
    private static Logger logger = new Logger();
    private static String conStatus = "No connection";

    private static JButton btn = new JButton("click me");
    private static JButton btn2 = new JButton("click me2");

    BufferedImage compassImg = null;
    BufferedImage backgroundImg = null;
    BufferedImage airspeedImg = null;
    BufferedImage elevationImg = null;
    BufferedImage headingImg = null;
    BufferedImage pointDImg = null;
    BufferedImage pointLImg = null;
    BufferedImage pointRImg = null;
    BufferedImage rollImg = null;
    BufferedImage yawMappedImg = null;
    BufferedImage pitchImg = null;
    AffineTransform tx = AffineTransform.getRotateInstance(metrics.getHeading(), 150, 150);
    AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);


    public JFrameGUI() {
        //load images
        compassImg = loadImg("images/compass.png");
        backgroundImg = loadImg("images/GUI.png");
        airspeedImg = loadImg("images/airspeed.png");
        elevationImg = loadImg("images/elevation.png");
        headingImg = loadImg("images/yaw.png");
        pointDImg = loadImg("images/pointD.png");
        pointLImg = loadImg("images/pointL.png");
        pointRImg = loadImg("images/pointR.png");
        rollImg = loadImg("images/roll.png");
        yawMappedImg = loadImg("images/yawMapped.png");
        pitchImg = loadImg("images/pitch.png");

        //init JFrame
        window = new JFrame();
        window.setSize(1200, 1050);
        window.setResizable(false);
        window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        window.setFocusable(true);
        window.setLocationRelativeTo(null);
        window.setTitle("Drone UI");

        //create options menu
        JMenuBar menuBar = new JMenuBar();

        JMenu menu = new JMenu("Options"); menu.setMnemonic(KeyEvent.VK_A);
        menu.getAccessibleContext().setAccessibleDescription("Options and preferences");
        menuBar.add(menu);
        JMenuItem menuItem;
        menuItem = new JMenuItem("Preferences", KeyEvent.VK_G);
        menuItem.addActionListener(this); menuItem.setActionCommand("Preferences");
        menu.add(menuItem);
        menuItem = new JMenuItem("Reconnect", KeyEvent.VK_G);
        menuItem.addActionListener(this); menuItem.setActionCommand("Reconnect");
        menu.add(menuItem);

        //create replay menu
        menu = new JMenu("Replay"); menu.setMnemonic(KeyEvent.VK_D);
        menu.getAccessibleContext().setAccessibleDescription("Replay flight logs");
        menuBar.add(menu);
        menuItem = new JMenuItem("Load logfile", KeyEvent.VK_T);
        menuItem.addActionListener(this); menuItem.setActionCommand("Load logfile");
        menu.add(menuItem);
        menuItem = new JMenuItem("Go to time", KeyEvent.VK_B);
        menuItem.addActionListener(this); menuItem.setActionCommand("Jump");
        menu.add(menuItem);
        menuItem = new JMenuItem("Go to next logline", KeyEvent.VK_H);
        menuItem.addActionListener(this); menuItem.setActionCommand("Next logline");
        menu.add(menuItem);
        JMenuItem menuItemPause = new JMenuItem("Pause", KeyEvent.VK_B);
        menuItemPause.addActionListener(this); menuItemPause.setActionCommand("Pause");
        menu.add(menuItemPause);
        JMenuItem menuItemPlay = new JMenuItem("Play", KeyEvent.VK_S);
        menuItemPlay.addActionListener(this); menuItemPlay.setActionCommand("Play");
        menu.add(menuItemPlay);


//        //create test panel
//        JPanel controlPanel = new JPanel(new GridBagLayout());
//        controlPanel.setPreferredSize(new java.awt.Dimension(1000, 500));
//        GridBagConstraints controlConstraints = new GridBagConstraints();
//        controlConstraints.anchor = GridBagConstraints.NORTH;
//        controlConstraints.insets = new Insets(1, 1, 1, 1);
//        controlConstraints.gridx = 0;
//        controlConstraints.gridy = 0;
//        controlPanel.add(btn, controlConstraints);
//        controlPanel.setOpaque(false);
//        add(controlPanel);
//
//        JPanel controlPanel2 = new JPanel(new GridBagLayout());
//        controlPanel2.setPreferredSize(new java.awt.Dimension(1000, 500));
//        GridBagConstraints control2Constraints = new GridBagConstraints();
//        control2Constraints.anchor = GridBagConstraints.NORTH;
//        control2Constraints.insets = new Insets(1, 1, 1, 1);
//        control2Constraints.gridx = 0;
//        control2Constraints.gridy = 0;
//        controlPanel2.add(btn2, control2Constraints);
//        add(controlPanel2);

        //add menu to jframe and set jframe to visible
        window.setJMenuBar(menuBar);
        window.setVisible(true);
        window.add(this);
        logger = new Logger();
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        //draw yaw circle
        tx = AffineTransform.getRotateInstance(Math.toRadians (360 - metrics.getYaw()), 250, 250); //rotates coordinates around an anchor point
        op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
        g2d.drawImage(op.filter(headingImg, null),587,387,null);
        g2d.drawImage(pointDImg,823,374,null);

        //draw yaw, pitch and roll
        //pitch
        float pitch = metrics.getPitch();
        int yOffset = (20/10) * (int)pitch;
        g2d.drawImage(pitchImg,634,103,1034,303,0,400-yOffset,400,600-yOffset,null);

        //roll
        tx = AffineTransform.getRotateInstance(Math.toRadians (metrics.getRoll()), 100, 100); //rotates coordinates around an anchor point
        op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
        g2d.drawImage(op.filter(rollImg, null),734,103,null);

        //yaw
        float yaw = metrics.getYaw();
        int xOffset = (200/180) * (int)yaw;
        g2d.drawImage(yawMappedImg,634,153,1034,253,200+xOffset,0,600+xOffset,100,null);




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
        float airspeed = metrics.getAirspeed(); //
        g2d.drawImage(airspeedImg, 530, 100, 580, 400, 20, 2499, 70, 2800,null);
        g2d.drawImage(pointLImg,565,233,null);

        //draw altidude meter
        float elevation = metrics.getElevation(); //
        g2d.drawImage(elevationImg, 1085, 100, 1135, 400, 0, 2499, 50, 2800, null);
        g2d.drawImage(pointRImg,1068,234,null);

        //draw elapsed time
        g2d.setColor(Color.YELLOW);
        g2d.drawString("T+" + logger.getTimestamp(), 50, 487);
        g2d.drawString("Connection status: " + conStatus, 50, 502);

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

    public void setConnectionStatus(String status) {
        this.conStatus = status;
    }

    public void actionPerformed(ActionEvent e) {
        if ("Preferences".equals(e.getActionCommand())) {
        }
        else if ("Reconnect".equals(e.getActionCommand())) {
            SerialController controller = new SerialController();
            controller.initialize();
        }
        else if ("Load logfile".equals(e.getActionCommand())) {
        }
        else if ("Jump".equals(e.getActionCommand())) {
        }
        else if ("Next logline".equals(e.getActionCommand())) {
        }
        else if ("Pause".equals(e.getActionCommand())) {
        }
        else if ("Play".equals(e.getActionCommand())) {
        }
    }

}
