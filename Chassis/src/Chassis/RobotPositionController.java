package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

  /**
   * <p>The whole-robot velocity controller.</p>
   **/
  protected RobotVelocityController robotVelocityController;

  /**
   * <p>Total ticks since reset, positive means corresp side of robot moved
   * forward.</p>
   **/
  protected double[] totalTicks = new double[2];

  /**
   * <p>Total elapsed time since reset in seconds.</p>
   **/
  protected double totalTime = 0.0;

  /**
   * <p>Time in seconds since last update.</p>
   **/
  protected double sampleTime;

  /**
   * <p>An abstract gain; meaning depends on the particular subclass
   * implementation.</p>
   **/
  protected double gain = 1.0;

  protected final static double DISTANCE_BETWEEN_WHEELS = 0.430;

  /**
   * <p>The robot.</p>
   **/
  protected OdometryRobot robot;

  /**
   * <p>position state variables.</p>
   **/
  protected double x = 0;
  protected double y = 0;
  protected double theta = 0;

  /**
   * <p>Create a new position controller for a robot.</p>
   *
   * @param robot the robot, not null
   **/
  public RobotPositionController(OdometryRobot robot) {
    this.robot = robot;
  }

  /**
   * <p>Translate at the specified speed for the specified distance.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in m/s
   * @param distance the desired distance to move in meters, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean translate(double speed, double distance) {
		boolean ok = true;
    // Begin Student Code
		
		//translation, Feed-Forward implementation
		
		double angularVelocityDesired=speed/WheelVelocityController.WHEEL_RADIUS_IN_M;
		double distAfterTranslating=distance+totalTicks[RobotBase.LEFT]/WheelVelocityController.TICKS_PER_REVOLUTION * 2 * Math.PI * WheelVelocityController.WHEEL_RADIUS_IN_M;
		
		//Set desired angular velocity
		robotVelocityController.setDesiredAngularVelocity(angularVelocityDesired,angularVelocityDesired);
		
		while (distAfterTranslating>totalTicks[RobotBase.LEFT]/WheelVelocityController.TICKS_PER_REVOLUTION * 2 * Math.PI * WheelVelocityController.WHEEL_RADIUS_IN_M){
			
		}
		//Set angular velocity to 0
		robotVelocityController.setDesiredAngularVelocity(0,0);
		
    // End Student Code
		return ok;
  }

  /**
   * <p>Rotate at the specified speed for the specifietotalTicks[RobotBase.LEFT]d angle.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in radians/s
   * @param angle the desired angle to rotate in radians, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean rotate(double speed, double angle) {
		boolean ok = true;
    // Begin Student Code
    // End Student Code
	  return ok;	
  }
    

    

  /**
   * <p>If position control is closed-loop, this computes the new left and
   * right velocity commands and issues them to {@link
   * #robotVelocityController}.</p>
   **/
  public synchronized void controlStep() {

    if (robotVelocityController == null)
      return;

    if (!robot.motorsEnabled() || robot.estopped())
      return;

    // Begin Student Code (if implementing closed-loop control)
    // End Student Code (if implementing closed-loop control)
  }

  /**
   * <p>Set the whole-robot velocity controller.</p>
   *
   * <p>This is called automatically by {@link OdometeryRobot}.</p>
   *
   * @param vc the whole-robot velocity controller
   **/
  public void setRobotVelocityController(RobotVelocityController vc) {
    robotVelocityController = vc;
  }

  /**
   * <p>Set {@link #gain}.</p>
   *
   * @param g the new gain
   **/
  public void setGain(double g) {
    gain = g;
  }

  /**
   * <p>Get {@link #gain}.</p>
   *
   * @return gain
   **/
  public double getGain() {
    return gain;
  }

  /**
   * <p>Update feedback and sample time.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   * @param leftTicks left encoder ticks since last update, positive means
   * corresp side of robot rolled forward
   * @param rightTicks right encoder ticks since last update, positive means
   * corresp side of robot robot rolled forward
   **/
  public synchronized void update(double time,
                                  double leftTicks, double rightTicks) {

    sampleTime = time;

    totalTicks[RobotBase.LEFT] += leftTicks;
    totalTicks[RobotBase.RIGHT] += rightTicks;
    totalTime += time;
    
    //Convert from ticks to meters
    double rightDist = rightTicks / WheelVelocityController.TICKS_PER_REVOLUTION * 2 * Math.PI * WheelVelocityController.WHEEL_RADIUS_IN_M;
    double leftDist = leftTicks / WheelVelocityController.TICKS_PER_REVOLUTION * 2 * Math.PI * WheelVelocityController.WHEEL_RADIUS_IN_M;

    //Useful definitions
    double distDiff = rightDist - leftDist;
    double distAvg = (rightDist + leftDist) / 2;
    
    //Calculate dtheta
    double dtheta = distDiff / DISTANCE_BETWEEN_WHEELS;

    //update our current odometry
    double thetaNew = theta + dtheta
    double thetaTravel = theta + dtheta / 2;
    double xNew = x + Math.sin(thetaTravel) * distAvg;
    double yNew = y + Math.cos(thetaTravel) * distAvg;
    
    //apply the new odometry
    x = xNew;
    y = yNew;
    theta = thetaNew;
  }
}
