package Braitenberg;

import MotorControlSolution.*;
//import MotorControl.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

/**
 * <p>Entry point for light sensors lab.</p>
 **/
public class LightSensors extends JFrame implements ActionListener {

  /**
   * <p>The robot.</p>
   **/
  public static RobotBase robot = new RobotBase();

  /**
   * <p>Port of the (robot) right photocell.</p>
   **/
  public static final int RIGHT_PHOTOCELL = 0;

  /**
   * <p>Port of the (robot) left photocell.</p>
   **/
  public static final int LEFT_PHOTOCELL = 7;
  
  /**
   * <p>The different kinds of behaviors that exist.</p>
   */
  public enum Behaviors {SEARCH, TWO_A, TWO_B, THREE_A, THREE_B, CREATIVE};
  
  /**
   * <p>GUI behavior chooser.</p>
   */
  private static JComboBox chooser;

  /**
   * <p>The behavior object.</p>
   */
  private static Behavior behavior;
  
  /**
   * <p>Constructor.  Sets up the simple GUI allowing you to
   * select which behavior you want.</p>
   */
  public LightSensors() {
	  this.setSize(400,200);
	  this.setMinimumSize(new Dimension(400,200));
	  this.setMaximumSize(new Dimension(400,200));
	  setupWidgets();        
	  this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	  this.pack();
	  this.setVisible(true);
  }
  
  /**
   * <p>Sets up and places the GUI's label and combobox.</p>
   */
  private void setupWidgets() {
	  JLabel lbl = new JLabel("Select your behavior:");
	  String[] behavior_str = { "Search and Find",
			  					"Braitenburg 2A",
			  					"Braitenburg 2B",
			  					"Braitenburg 3A",
			  					"Braitenburg 3B",
			  					"Student Soln"};
	  chooser = new JComboBox(behavior_str);
	  chooser.addActionListener(this);
	  
	  //TODO: Change this to the default behavior you would like!
	  chooser.setSelectedItem("Braitenburg 2A");
	  
	  JPanel content = new JPanel();
	  content.add(lbl);
	  content.add(chooser);
	  this.setContentPane(content);
  }

  /**
   * <p>Entry point for the light sensors lab.</p>
   *
   * @param args command line arguments
   **/
  public static void main(String [] args) {
	  new LightSensors();
	  
    /////////////////// create velocity controller //////////////////////    

    RobotVelocityController robotVelocityController =
      new RobotVelocityController(new WheelVelocityControllerI(), new WheelVelocityControllerI());

    robot.setRobotVelocityController(robotVelocityController);

    //////////////////// config controllers /////////////////////////////    

    //if your velocity controller needs to be configured
    //(i.e. gains set, etc), then do it here

    //this block to configures *our* solution to lab 2 (yours may or
    //may not be configured the same way)
    final double VELOCITY_BALANCE_GAIN = 1.0; 
    final double VELOCITY_WHEEL_GAIN = 6.0;
    robotVelocityController.setGain(VELOCITY_BALANCE_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.LEFT). 
      setGain(VELOCITY_WHEEL_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.RIGHT).
      setGain(VELOCITY_WHEEL_GAIN);

    //////////////////// display estop button //////////////////////////    

    EstopButton estop = new EstopButton(Thread.currentThread());

    //////////////////// command motion ////////////////////////////////    

    //create and calibrate light sensors, will take 5s per sensor
    Photocell leftPhotocell = new Photocell(robot, LEFT_PHOTOCELL);
    Photocell rightPhotocell = new Photocell(robot, RIGHT_PHOTOCELL);
    leftPhotocell.calibrate();
    rightPhotocell.calibrate();

    //create and execute behavior
    behavior = new Behavior(robot, leftPhotocell, rightPhotocell, getBehaviorsFromString(chooser.getSelectedItem().toString()));
    behavior.go();
    

    //////////////////// shutdown //////////////////////////////////////    

    //robot should already be stopped, but just in case
    robot.estop();
    System.exit(0);
    
  }

  /**
   * <p>The estop button.</p>
   **/
  protected static class EstopButton extends JDialog {
    
    EstopButton(final Thread mainThread) {

      JButton eb = new JButton("ESTOP");
      eb.setBackground(Color.RED);
      eb.setPreferredSize(new Dimension(200, 200));
      eb.addActionListener(new ActionListener() {
          public void actionPerformed(ActionEvent e) {
            mainThread.interrupt();
            robot.estop();
            System.exit(-1);
          }
        });

      setContentPane(eb);
      setTitle("Emergency Stop");
      setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
      pack();
      setVisible(true);
    }
  }
  
  private static Behaviors getBehaviorsFromString(String str) {
	  if (str.equalsIgnoreCase("Search and Find")) {
		  return Behaviors.SEARCH;
	  }
	  else if (str.equalsIgnoreCase("Braitenburg 2A")) {
		  return Behaviors.TWO_A;
	  }
	  else if (str.equalsIgnoreCase("Braitenburg 2B")) {
		  return Behaviors.TWO_B;
	  }
	  else if (str.equalsIgnoreCase("Braitenburg 3A")) {
		  return Behaviors.THREE_A;
	  }
	  else if (str.equalsIgnoreCase("Braitenburg 3B")) {
		  return Behaviors.THREE_B;		  
	  }
	  else if (str.equalsIgnoreCase("Student Soln")) {
		  return Behaviors.CREATIVE;
	  }
	  else return null;
  }

  public void actionPerformed(ActionEvent e) {
	  if (behavior != null)
		  behavior.setBehavior(getBehaviorsFromString(chooser.getSelectedItem().toString()));	  
  }
} 
