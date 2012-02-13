package MotorControl;

import java.awt.*;
import java.awt.event.*;

import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;
import javax.swing.event.*;

import java.text.DecimalFormat;

/**
 * <p>GUI for motor control.</p>
 *
 * @author Aisha Walcott
 * @author vona
 **/
public class MotorControlGUI extends JFrame implements WindowListener {

  public static final int GAIN_RES = 100;
  public static final int DIFF_RES = 100;
  public static final int DES_RES = 100;

  DecimalFormat df = new DecimalFormat() {
        { setMaximumFractionDigits(2); }
    };
  
  protected RobotBase robot;
  protected RobotVelocityController rvc;
  
  protected JButton systemButton;
  protected JLabel systemLabel;
  protected boolean motorsEnabled = false;

  protected JButton dataLogButton;
  protected JLabel loggingLabel;
  protected boolean logging = false;

  protected Timer graphTimer;

  protected double diffV = 0.0;
  
  WheelPanel[] wheelPanel = new WheelPanel[2];

  public MotorControlGUI(RobotBase robot) {
    
    this.robot = robot;

    rvc = robot.getRobotVelocityController();
    
    if (rvc == null) {
      System.err.println("Must set a RobotVelocityController!");
      System.exit(-1);
    }
    
    Box topBox = Box.createVerticalBox();

    if (rvc instanceof RobotVelocityControllerBalanced) {
      topBox.add(setupRobotPanel());
      topBox.add(Box.createVerticalStrut(5));
    }

    wheelPanel[RobotBase.LEFT] = new WheelPanel("Left", RobotBase.LEFT);
    wheelPanel[RobotBase.RIGHT] = new WheelPanel("Right", RobotBase.RIGHT);
    
    Box wheelsBox = Box.createHorizontalBox();
    wheelsBox.add(wheelPanel[RobotBase.LEFT]);
    wheelsBox.add(Box.createHorizontalStrut(5));
    wheelsBox.add(wheelPanel[RobotBase.RIGHT]);

    topBox.add(wheelsBox);
    
    JPanel bottomPanel = setupBottomPanel();
    
    Container cp = getContentPane();
    cp.setLayout(new BorderLayout());
    
    cp.add(topBox, BorderLayout.CENTER);
    cp.add(bottomPanel, BorderLayout.SOUTH);
    
    this.addWindowListener(this);
    setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
    pack();
    
    setTitle("Motor Control GUI");
    setSize(900, 900);
    setVisible(true);
  }

  protected Component setupRobotPanel() {

    final JSlider diffSlider = new JSlider(-10*DIFF_RES, 10*DIFF_RES, 0);
    final JTextField diffValue = new JTextField("0", 3);

    diffSlider.setMajorTickSpacing(5*DIFF_RES);
    diffSlider.setMinorTickSpacing(1*DIFF_RES);
    diffSlider.setPaintTicks(true);
    diffSlider.addChangeListener(new ChangeListener() {
        public void stateChanged(ChangeEvent e) {
          double diffVWas = diffV;
          double lwv = rvc.getDesiredAngularVelocity(RobotBase.LEFT);
          diffV = ((double) diffSlider.getValue())/DIFF_RES;
          diffValue.setText(df.format(diffV));
          wheelPanel[RobotBase.LEFT].
            setWheelVelocity(lwv+(diffV-diffVWas)/2.0);
        } });

    diffValue.setMaximumSize(diffValue.getPreferredSize());
    diffValue.addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          double diffVWas = diffV;
          double lwv = rvc.getDesiredAngularVelocity(RobotBase.LEFT);
          diffV = Double.parseDouble(diffValue.getText()); 
          diffSlider.setValue((int) (diffV*DIFF_RES));
          wheelPanel[RobotBase.LEFT].
            setWheelVelocity(lwv+(diffV-diffVWas)/2.0);
        } });
   
    Box diffBox = Box.createHorizontalBox();
    diffBox.add(new JLabel("Diff Vel: "));
    diffBox.add(diffSlider);
    diffBox.add(diffValue);
    diffBox.add(new JLabel("rad/s"));

    final JSlider gainSlider = new JSlider(-30*GAIN_RES, 30*GAIN_RES, 0);
    final JTextField gainValue = new JTextField("0", 3);

    gainSlider.setMajorTickSpacing(10*GAIN_RES);
    gainSlider.setMinorTickSpacing(5*GAIN_RES);
    gainSlider.setPaintTicks(true);
    gainValue.setMaximumSize(gainValue.getPreferredSize());

    double g = rvc.getGain();
    gainSlider.setValue((int) (g*GAIN_RES));
    gainValue.setText(df.format(g));

    gainSlider.addChangeListener(new ChangeListener() {
        public void stateChanged(ChangeEvent e) {
          double gain = ((double) gainSlider.getValue())/GAIN_RES;
          gainValue.setText(df.format(gain));
          rvc.setGain(gain);
        } });

    gainValue.addActionListener(new ActionListener() {
        public void actionPerformed(ActionEvent e) {
          double gain = Double.parseDouble(gainValue.getText());
          gainSlider.setValue((int) (gain*GAIN_RES));
          rvc.setGain(gain);
        } });
    
    Box gainBox = Box.createHorizontalBox();
    gainBox.add(new JLabel("RVC Gain : "));
    gainBox.add(gainSlider);
    gainBox.add(gainValue);

    Box box = Box.createVerticalBox();
    box.add(diffBox);
    box.add(gainBox);
    
    box.setBorder(new TitledBorder(new EtchedBorder(), "Whole Robot"));

    return box;
  }

  protected JPanel setupBottomPanel() {

    systemButton = new JButton("Start");
    systemButton.setPreferredSize(new java.awt.Dimension(90, 20));
    systemLabel = new JLabel("System halted", JLabel.CENTER);
    systemLabel.setForeground(Color.yellow);
    systemButton.addActionListener(new SystemButtonListener());
    
    JPanel systemPanel = new JPanel(new FlowLayout(FlowLayout.LEADING));
    systemPanel.add(systemButton);
    systemPanel.add(systemLabel);
    systemPanel.setBackground(Color.black);
    
    loggingLabel = new JLabel();
    loggingLabel.setForeground(Color.yellow);
    dataLogButton = new JButton("Start Log");
    dataLogButton.setPreferredSize(new java.awt.Dimension(120,20));
    dataLogButton.addActionListener(new DataLogListener());
    
    JPanel dataPanel = new JPanel(new FlowLayout(FlowLayout.LEADING));
    dataPanel.add(dataLogButton);//change to a button
    dataPanel.add(loggingLabel);
    dataPanel.setBackground(Color.black);
    
    Box low = Box.createVerticalBox();
    low.setBorder(new CompoundBorder(
                    BorderFactory.createLineBorder(Color.black, 1),
                    BorderFactory.createBevelBorder(BevelBorder.RAISED)));
    low.add(dataPanel);
    low.add(systemPanel);

    JPanel bottomPanel = new JPanel(new BorderLayout());
    bottomPanel.add(low, BorderLayout.CENTER );
    return bottomPanel;
  }
    
  class DataLogListener implements ActionListener {
    public void actionPerformed(ActionEvent e) {
      if (!logging) {
        dataLogButton.setText("Stop Log");
        loggingLabel.setText("Logging data . . .");
        robot.startVelocityLog();
        logging = true;
      } else {
        robot.stopVelocityLog();
        dataLogButton.setText("Start Log");
        loggingLabel.setText("");
        logging = false;
      }
    }
  }
  
  class SystemButtonListener implements ActionListener {
    public void actionPerformed(ActionEvent e) {
      if (motorsEnabled == false) {
          systemButton.setText("Stop");
          systemLabel.setText("System running");
          robot.enableMotors(true);
          motorsEnabled = true;
      } else {
        systemButton.setText("Start");
        systemLabel.setText("System halted");
        robot.enableMotors(false);
        motorsEnabled = false;
      }
    }
  }

  class WheelPanel extends JPanel {

    protected int wheel;
    protected WheelVelocityController wvc;

    protected Box displayBox;

    protected WheelPanelListener wpListener;
    
    protected JRadioButton manualRadioButton =
      new JRadioButton("Manual", false);
    protected JRadioButton stepRadioButton = new JRadioButton("Step", false);
    protected JRadioButton sineRadioButton = new JRadioButton("Sine", false);
    
    protected Timer inputFunctionTimer;
    
    protected JSlider desiredSlider =
      new JSlider(-255*DES_RES, 255*DES_RES, 0);
    protected JTextField desiredValue = new JTextField("0", 3);
    protected JPanel desiredSliderPanel =
      new JPanel(new FlowLayout(FlowLayout.LEADING));
    protected JLabel desiredLabel = new JLabel("");
    protected JLabel desiredUnitsLabel = new JLabel("");
    
    protected JSlider gainSlider = new JSlider(-30*GAIN_RES, 30*GAIN_RES, 0);
    protected JTextField gainValue = new JTextField("1.00", 5);
    protected JPanel gainPanel =
      new JPanel(new FlowLayout(FlowLayout.LEADING));
    
    protected JLabel pwmValue;
    protected JLabel angularVelocityValue;

    protected GraphPanel graphPanel;
    protected Timer graphTimer;

    public WheelPanel(String side, int wheel) {
      super(new BorderLayout());
      
      this.wheel = wheel;
   
      wvc = rvc.getWheelVelocityController(wheel);

      String title = side + " Wheel: " + wvc.getName();
      
      wpListener = new WheelPanelListener();
      
      displayBox = Box.createVerticalBox();
      displayBox.setPreferredSize(new java.awt.Dimension(400,200));
      displayBox.add(Box.createVerticalStrut(10));
      
      addChoicePanel();
      addChoiceActionListeners();

      addDesiredSliderPanel();
      addGainSliderPanel();
      
      pwmValue = new JLabel();
      angularVelocityValue = new JLabel();
      
      addReadOnlyValuePanel("PWM:", pwmValue, "");
      addReadOnlyValuePanel("Ang Vel:", angularVelocityValue, "rad/s");
          
      graphPanel = new GraphPanel(400,200,200);//w,h,number of x values
      displayBox.add(graphPanel);
      
      setBorder(new TitledBorder(new EtchedBorder(), title));
      add(displayBox, BorderLayout.CENTER );

      inputFunctionTimer = new Timer(50, new InputFunctionActionListener());
      inputFunctionTimer.start();
      
      graphTimer = new Timer(50, new GraphListener());
      graphTimer.start();
    }
    
    class GraphListener implements ActionListener {
      public void actionPerformed(ActionEvent e) {
        pwmValue.setText(Integer.toString(robot.getPWMGoal(wheel)));
        double v = rvc.computeAngularVelocity(wheel);
        angularVelocityValue.setText(df.format(v));
        graphPanel.log(v);
      }
    }
    
    protected void addReadOnlyValuePanel(String title,
                                       JLabel value,
                                       String units) {
      JPanel labelPanel = new JPanel(new FlowLayout(FlowLayout.LEADING));
      labelPanel.add(new JLabel(title));
      labelPanel.add(value);
      labelPanel.add(new JLabel(units));
      displayBox.add(labelPanel);
      displayBox.add(Box.createVerticalStrut(5));
    }
    
    protected void addDesiredSliderPanel() {

      desiredSlider.setMajorTickSpacing(25*DES_RES);
      desiredSlider.setMinorTickSpacing(5*DES_RES);
      desiredSlider.setPaintTicks(true);

      desiredSlider.addChangeListener(new DesiredSliderListener());
      desiredValue.addActionListener(wpListener);

      desiredSliderPanel.add(desiredLabel);
      desiredSliderPanel.add(desiredSlider);
      desiredSliderPanel.add(desiredValue);
      desiredSliderPanel.add(desiredUnitsLabel);

      displayBox.add(desiredSliderPanel);
      displayBox.add(Box.createVerticalStrut(5));
    }
    
    protected void addGainSliderPanel() {
      
      gainSlider.setMajorTickSpacing(10*GAIN_RES);
      gainSlider.setMinorTickSpacing(5*GAIN_RES);
      gainSlider.setPaintTicks(true);

      double g = wvc.getGain();
      gainSlider.setValue((int) (g*GAIN_RES));
      gainValue.setText(df.format(g));
      
      gainSlider.addChangeListener(new GainSliderListener());
      gainValue.addActionListener(wpListener);

      gainPanel.add(new JLabel("Wheel Gain: "));
      gainPanel.add(gainSlider);
      gainPanel.add(gainValue);
      
      displayBox.add(gainPanel);
      displayBox.add(Box.createVerticalStrut(5));
    }
    
    protected void addChoicePanel() {

      JPanel choicePanel = new JPanel(new FlowLayout(FlowLayout.LEADING));
      choicePanel.add(new JLabel("Input Function: "));

      if (wvc instanceof WheelVelocityControllerPWM) {
        choicePanel.add(manualRadioButton);
      } else {
        choicePanel.add(manualRadioButton);
        choicePanel.add(stepRadioButton);
        choicePanel.add(sineRadioButton);
      }

      manualRadioButton.setSelected(true);
      manualRadioButtonAction();

      displayBox.add(choicePanel);
    }
    
    protected void manualRadioButtonAction() {

      manualRadioButton.setSelected(true);
      stepRadioButton.setSelected(false);
      sineRadioButton.setSelected(false);

      desiredSlider.setEnabled(true);
      desiredValue.setEnabled(true);

      if (wvc instanceof WheelVelocityControllerPWM) {

        desiredLabel.setText("Desired PWM");
        desiredUnitsLabel.setText("PWM");
        desiredSlider.setMinimum(-255*DES_RES);
        desiredSlider.setMaximum(255*DES_RES);
        desiredSlider.setValue(0);
        desiredSlider.setMajorTickSpacing(25*DES_RES);
        desiredSlider.setMinorTickSpacing(5*DES_RES);

        gainSlider.setEnabled(false);
        gainValue.setEnabled(false);

      } else {

        desiredLabel.setText("Desired Velocity");
        desiredUnitsLabel.setText("rad/s");
        desiredSlider.setMinimum(-10*DES_RES);
        desiredSlider.setMaximum(10*DES_RES);
        desiredSlider.setValue(0);
        desiredSlider.setMajorTickSpacing(5*DES_RES);
        desiredSlider.setMinorTickSpacing(1*DES_RES);

        gainSlider.setEnabled(true);
        gainValue.setEnabled(true);
      }
    }
        
    protected void addChoiceActionListeners() {

      manualRadioButton.addActionListener(wpListener);
      manualRadioButton.addActionListener(new ActionListener(){
          public void actionPerformed(ActionEvent e){
            manualRadioButtonAction();
          } });

      stepRadioButton.addActionListener(wpListener);
      stepRadioButton.addActionListener(new ActionListener(){
          public void actionPerformed(ActionEvent e){

            manualRadioButton.setSelected(false);
            stepRadioButton.setSelected(true);
            sineRadioButton.setSelected(false);

            desiredSlider.setEnabled(false);
            desiredValue.setEnabled(false);

            gainSlider.setEnabled(true);
            gainValue.setEnabled(true);

          } });

      sineRadioButton.addActionListener(wpListener);
      sineRadioButton.addActionListener(new ActionListener(){
          public void actionPerformed(ActionEvent e){

            manualRadioButton.setSelected(false);
            stepRadioButton.setSelected(false);
            sineRadioButton.setSelected(true);

            desiredSlider.setEnabled(false);
            desiredValue.setEnabled(false);

            gainSlider.setEnabled(true);
            gainValue.setEnabled(true);

          } });
    }
    
    class InputFunctionActionListener implements ActionListener{
      
      protected double t = 0, v = 0;
      protected double vSign = 1;

      public void actionPerformed(ActionEvent e) {

        if (sineRadioButton.isSelected()) {

          v = 8 * Math.sin(t);
          t = t + (Math.PI / 40);

        } else if (stepRadioButton.isSelected()) {

          if (t > 100) {

            v = v + 3*vSign;

            if (v > 10)
              vSign = -1;

            if (v < -10)
              vSign = 1;

            t = 0;

          } else {
            t = t + 1;
          }
        }

        if (!manualRadioButton.isSelected()) {
          setWheelVelocity(v);
        }
      }
    }
   
    protected void setWheelVelocity(double v) {

      if (rvc instanceof RobotVelocityControllerBalanced) {
       
        int otherWheel = RobotBase.otherWheel(wheel);

        double otherV = v + diffV*((wheel == RobotBase.LEFT) ? -1.0 : 1.0);

        WheelPanel otherPanel = wheelPanel[otherWheel];

        otherPanel.desiredSlider.setValue((int) (otherV*DES_RES));
        otherPanel.desiredValue.setText(df.format(otherV));
       
        double lv = (wheel == RobotBase.LEFT) ? v : otherV;
        double rv = (wheel == RobotBase.RIGHT) ? v : otherV;

        rvc.setDesiredAngularVelocity(lv, rv);

      } else {

        desiredSlider.setValue((int) (v*DES_RES));
        desiredValue.setText(df.format(v));

        rvc.setDesiredAngularVelocity(wheel, v);
      }
    }
   
    protected void setGain(double g) {
      
      gainSlider.setValue((int) (g*GAIN_RES));
      gainValue.setText(df.format(g));

      if (rvc instanceof RobotVelocityControllerBalanced) {
        
        WheelPanel other = wheelPanel[RobotBase.otherWheel(wheel)];
        
        other.gainSlider.setValue((int) (g*GAIN_RES));
        other.gainValue.setText(df.format(g));
       
        other.wvc.setGain(g);
      } 

      wvc.setGain(g);
    }

    class WheelPanelListener implements ActionListener {
      public void actionPerformed(ActionEvent e) {
        setGain(Double.parseDouble(gainValue.getText()));
        setWheelVelocity(Double.parseDouble(desiredValue.getText()));
      }
    }
        
    class DesiredSliderListener implements ChangeListener{
      public void stateChanged(ChangeEvent e) {
        setWheelVelocity(((double) (desiredSlider.getValue()))/DES_RES);
      }
    }
    
    class GainSliderListener implements ChangeListener {
      public void stateChanged(ChangeEvent e) {
        setGain(((double) (gainSlider.getValue()))/GAIN_RES);
      }
    }
  }
  public void windowClosing(WindowEvent arg0) {
	  /*
      robot.enableMotors(false);
      motorsEnabled = false;*/
	  robot.estop();
      System.out.println("******* window closing ********");
      this.dispose();
      System.exit(0);
  }
  // methods for WindowListener.  Should stay empty.
  public void windowOpened(WindowEvent arg0) {}
  public void windowClosed(WindowEvent arg0) {}
  public void windowIconified(WindowEvent arg0) {}
  public void windowDeiconified(WindowEvent arg0) {}
  public void windowActivated(WindowEvent arg0) {}
  public void windowDeactivated(WindowEvent arg0) {}
}

