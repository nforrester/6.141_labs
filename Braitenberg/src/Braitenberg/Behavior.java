package Braitenberg;

import MotorControlSolution.*;
import Braitenberg.LightSensors.Behaviors;
//import MotorControl.RobotBase;
//import MotorControl.RobotVelocityController;

/**
 * <p>Implementation of Braitenberg and related behaviors.</p>
 **/
public class Behavior {

  /**
   * <p>Update period between behavior iterations in ms.</p>
   **/
  public static final long BEHAVIOR_UPDATE_PERIOD_MS = 5;


  /**
   * <p>The robot.</p>
   **/
  protected RobotBase robot;

  /**
   * <p>The robot velocity controller.</p>
   **/
  protected RobotVelocityController robotVelocityController;

  /**
   * <p>(robot) left photocell.</p>
   **/
  protected Photocell leftPhotocell;

  /**
   * <p>(robot) right photocell.</p>
   **/
  protected Photocell rightPhotocell;
  
  /**
   * <p>The current behavior.</p>
   */
  protected Behaviors currentBehavior;
   
  /**
   * <p>Create a new behavior object.</p>
   *
   * @param robot the robot
   * @param leftPhotocell the (robot) left light sensor, already calibrated
   * @param rightPhotocell the (robot) right light sensor, already calibrated
   **/
  public Behavior(RobotBase robot,
                  Photocell leftPhotocell, Photocell rightPhotocell,
                  Behaviors behavior){
    this.robot = robot;
    this.robotVelocityController = robot.getRobotVelocityController();
    this.leftPhotocell = leftPhotocell;
    this.rightPhotocell = rightPhotocell;
    this.currentBehavior = behavior;
  }
  
  /**
   * <p>Sets the current behavior of the robot.</p>
   */
  public void setBehavior(Behaviors b) {
	  currentBehavior = b;
  }

  /**
   * <p>Convenience method to make a beep sound.</p>
   **/
  protected void beep() {
    java.awt.Toolkit.getDefaultToolkit().beep();
  }

  /**
   * <p>Convenience method to {@link
   * RobotVelocityController#setDesiredAngularVelocity} on {@link
   * #robotVelocityController}.</p>
   *
   * @param left the desired left wheel angular velocity in rad/s, positive
   * means corresp side moves forward
   * @param right the desired right wheel angular velocity in rad/s, positive
   * means corresp side moves forward
   **/
  protected void setDesiredAngularVelocity(double left, double right) {
    robotVelocityController.setDesiredAngularVelocity(left, right);
  }

  /**
   * <p>Execute the behavior.</p>
   *
   * <p>Repeatedly runs a selected behavior until the robot is estopped.</p>
   *
   * <p>Waits {@link #BEHAVIOR_UPDATE_PERIOD_MS} between updates.</p>
   **/
  public void go() {

    robot.enableMotors(true);

    boolean found = false;
    boolean arrived = false;

    for (;;) {
      
      if (robot.estopped())
        return;
      
      double l = leftPhotocell.getValue();
      double r = rightPhotocell.getValue();

      //select your behavior here
      if (currentBehavior == Behaviors.SEARCH) {
	      if (!found)
	        found |= search(l, r);
	
	      if (found && !arrived) 
	        arrived |= goToLight(l, r);    	  
      }
      else if (currentBehavior == Behaviors.TWO_A) {
    	  vehicle2a(l, r);    	  
      }
      else if (currentBehavior == Behaviors.TWO_B) {
    	  vehicle2b(l, r);
      }
      else if (currentBehavior == Behaviors.THREE_A) {
    	  vehicle3a(l, r);
      }
      else if (currentBehavior == Behaviors.THREE_B) {
    	  vehicle3b(l, r);
      }
      else if (currentBehavior == Behaviors.CREATIVE) {
      // Begin Student Code
      // End Student Code
      }

      try {
        Thread.sleep(BEHAVIOR_UPDATE_PERIOD_MS);
      } catch (InterruptedException e) {
        robot.estop();
        break;
      }
    }
  }
   
  /**
   * <p>Rotate in place to search for the light.</p>
   *
   * <p>Rotates until a light is found (according to some criteria).</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   *
   * @return true if the light has been found
   **/
  public boolean search(double l, double r) {
	boolean found = false;

	// Begin Student Code
	double possibilityThreshold = 10;
	double foundThreshold = 25;
	double sameness = 0.2;
	double angvel;

	System.out.println("(LR " + l + " " + r + ")");

	if (l < possibilityThreshold && r < possibilityThreshold) {
		System.out.println("We ain't found shit! Veer left! Left! Left!");
		setDesiredAngularVelocity(3, -3);
	} else if (!(l > foundThreshold && r > foundThreshold && Math.abs(l - r) < sameness)) {
		found = false;
		angvel = (r - l) / possibilityThreshold * 1;
		setDesiredAngularVelocity(angvel, -1 * angvel);
	} else {
		found = true;
		System.out.println("Found!");
		setDesiredAngularVelocity(0, 0);
	}
	// End Student Code

	return found;
  }

  /**
   * <p>Move in a straight line towards the light.</p>
   *
   * <p>Speed should decrease in proportion to distance from light.</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   *
   * @return true if the robot has arrived at the light
   **/
  public boolean goToLight(double l, double r) {
	// Begin Student Code
	double distancel = 100 - l;
	double distancer = 100 - r;
	double stopDistance = 50;
	System.out.println("(LRD " + distancel + " " + distancer + ")");
	if (distancel > stopDistance && distancer > stopDistance) {
		System.out.println("gangway!");
		setDesiredAngularVelocity(3, 3);
		return false;
	} else {
		System.out.println("halt!");
		setDesiredAngularVelocity(0, 0);
		return true;
	}
	// End Student Code
  }

  /**
   * <p>Braitenberg's vehicle 2a.</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   **/
  public void vehicle2a(double l, double r) {
	// Begin Student Code
	// End Student Code
  }

  /**
   * <p>Braitenberg's vehicle 2b.</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   **/
  public void vehicle2b(double l, double r) {
	// Begin Student Code
	// End Student Code
  }

  /**
   * <p>Braitenberg's vehicle 3a.</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   **/
  public void vehicle3a(double l, double r) {
	// Begin Student Code
	// End Student Code
  }

  /**
   * <p>Braitenberg's vehicle 3b.</p>
   *
   * @param l the (robot) left calibrated sensor reading
   * @param r the (robot) right calibrated sensor reading
   **/
  public void vehicle3b(double l, double r) {
	// Begin Student Code
	// End Student Code
  }

}

