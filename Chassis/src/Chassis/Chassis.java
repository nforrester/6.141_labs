package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

/**
 * <p>Entry point for chassis lab.</p>
 **/
public class Chassis {

  /**
   * <p>The robot.</p>
   **/
  public static OdometryRobot robot = new OdometryRobot();

  /**
   * <p>Entry point for the chassis lab.</p>
   *
   * @param args command line arguments
   **/
  public static void main(String [] args) {

    //////////////////// create velocity controller //////////////////////    

    RobotVelocityController robotVelocityController =
      new RobotVelocityControllerBalanced();

    robot.setRobotVelocityController(robotVelocityController);


    //////////////////// create position controller /////////////////////    

    RobotPositionController robotPositionController =
      new RobotPositionController(robot);

    robot.setRobotPositionController(robotPositionController);


    //////////////////// config controllers /////////////////////////////    

    //if your velocity and/or position controllers need to be configured
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

    // Begin Student Code
    // End Student Code

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
} 
