package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

	
	 /**
	   * <p>The maximum pwm command magnitude.</p>
	   **/
	  protected static final double MAX_PWM = 255;

	  /**
	   * <p>Student Code: unloaded maximum wheel angular velocity in rad/s. 
	   * This should be a protected static final double called MAX_ANGULAR_VELOCITY.</p>
	   **/
	  protected static final double MAX_ANGULAR_VELOCITY = 7.95;

	  /**
	   * <p>Student Code: radius of the wheel on the motor (in meters). 
	   * This should be a protected static final double called WHEEL_RADIUS_IN_M.</p>
	   **/
	  protected static final double WHEEL_RADIUS_IN_M = .062775;

	  /**
	   * <p>Student Code: encoder resolution.</p>
	   * This should be a protected static final double called ENCODER_RESOLUTION.</p>
	   **/
	  protected static final double ENCODER_RESOLUTION = 2000;

	  /**
	   * <p>Student Code: motor revolutions per wheel revolution.
	   * This should be a protected static final double called GEAR_RATIO.</p>
	   **/
	  protected static final double GEAR_RATIO = 65.5;

	  /**
	   * <p>Student Code: encoder ticks per motor revolution.</p>
	   * This should be a protected static final double called TICKS_PER_REVOLUTION.</p>
	   **/
	  protected static final double TICKS_PER_REVOLUTION = 131000;
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
		if (distance<0 && speed>0 || distance>0 && speed<0)
			speed=-speed;

		double kR=TICKS_PER_REVOLUTION * 2 * Math.PI * WHEEL_RADIUS_IN_M;
		
		double[] desiredPose={x+distance*Math.sin(theta),y+distance*Math.cos(theta),theta};
		double[] myPose={x,y,theta};
		
		double angularVelocityDesired=speed/WHEEL_RADIUS_IN_M;
		
		double distAfterTranslatingL=distance+totalTicks[RobotBase.LEFT]/kR;
		double distAfterTranslatingR=distance+totalTicks[RobotBase.RIGHT]/kR;
		
		//Set desired angular velocity
		robotVelocityController.setDesiredAngularVelocity(angularVelocityDesired,angularVelocityDesired);
		
		while (comparePose(myPose,desiredPose,.05)){
			
			//Do nothing until we get there, unless we get an error.
			//if one of our wheels overshoots the other, stop and return false;
			if((distAfterTranslatingL>totalTicks[RobotBase.LEFT]/kR
					&& distAfterTranslatingR<totalTicks[RobotBase.RIGHT]/kR)
					||(distAfterTranslatingL<totalTicks[RobotBase.LEFT]/kR
							&& distAfterTranslatingR>totalTicks[RobotBase.RIGHT]/kR)){
				robotVelocityController.setDesiredAngularVelocity(0,0);
				System.out.println("We didn't get there.");
				return false;
				
			}
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
  
  public void printPose(){
	  System.out.println("Current Pose: X: "+x+" Y: "+y+" Theta: "+theta);
  }
  
  /**
   * <p>Check if pose1 is near pose 2 within a certain tolerance</p>
   *
   * @param pose1 pose to check
   * @param pose2 pose to compare to
   * @param tolerance a percentage of tolerance (write as decimal)
   **/
  
  public boolean comparePose(double [] pose1, double [] pose2, double tolerance){
	  if (((pose1[0]<pose2[0]+pose2[0]*tolerance)||(pose1[0]>pose2[0]-pose2[0]*tolerance))
			  && ((pose1[1]<pose2[1]+pose2[1]*tolerance)||(pose1[1]>pose2[1]-pose2[1]*tolerance))
					  && ((pose1[2]<pose2[2]+pose2[2]*tolerance)||(pose1[2]>pose2[2]-pose2[2]*tolerance))){
		  return true;
	  }
	  else
		  return false;
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
    double rightDist = rightTicks / TICKS_PER_REVOLUTION * 2 * Math.PI * WHEEL_RADIUS_IN_M;
    double leftDist = leftTicks / TICKS_PER_REVOLUTION * 2 * Math.PI * WHEEL_RADIUS_IN_M;

    //Useful definitions
    double distDiff = rightDist - leftDist;
    double distAvg = (rightDist + leftDist) / 2;
    
    //Calculate dtheta
    double dtheta = distDiff / DISTANCE_BETWEEN_WHEELS;

    //update our current odometry
    double thetaNew = theta + dtheta;
    double thetaTravel = theta + dtheta / 2;
    double xNew = x + Math.sin(thetaTravel) * distAvg;
    double yNew = y + Math.cos(thetaTravel) * distAvg;
    
    //apply the new odometry
    x = xNew;
    y = yNew;
    theta = thetaNew;
  }
}
