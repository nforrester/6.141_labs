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
  protected static final double MAX_PWM = WheelVelocityController.MAX_PWM;

  /**
   * <p>Student Code: unloaded maximum wheel angular velocity in rad/s.
   * This should be a protected static final double called MAX_ANGULAR_VELOCITY.</p>
   **/
  protected static final double MAX_ANGULAR_VELOCITY = WheelVelocityController.MAX_ANGULAR_VELOCITY;

  /**
   * <p>Student Code: radius of the wheel on the motor (in meters).
   * This should be a protected static final double called WHEEL_RADIUS_IN_M.</p>
   **/
  protected static final double WHEEL_RADIUS_IN_M = WheelVelocityController.WHEEL_RADIUS_IN_M;

  /**
   * <p>Student Code: encoder resolution.</p>
   * This should be a protected static final double called ENCODER_RESOLUTION.</p>
   **/
  protected static final double ENCODER_RESOLUTION = WheelVelocityController.ENCODER_RESOLUTION;

  /**
   * <p>Student Code: motor revolutions per wheel revolution.
   * This should be a protected static final double called GEAR_RATIO.</p>
   **/
  protected static final double GEAR_RATIO = WheelVelocityController.GEAR_RATIO;

  /**
   * <p>Student Code: encoder ticks per motor revolution.</p>
   * This should be a protected static final double called TICKS_PER_REVOLUTION.</p>
   **/
  protected static final double TICKS_PER_REVOLUTION = WheelVelocityController.TICKS_PER_REVOLUTION;

  /**
   * <p>Student Code: encoder ticks per meter.</p>
   * ticks/rev * rev/rad * rad/m = ticks/m
   **/
  protected static final double TICKS_PER_METER = TICKS_PER_REVOLUTION * (1 / (2 * Math.PI)) * (1 / WHEEL_RADIUS_IN_M);

  /**
   * <p>The whole-robot velocity controller.</p>
   **/
  protected RobotVelocityControllerBalanced robotVelocityController;

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

	double[] desiredPose={x+distance*Math.cos(theta),y+distance*Math.sin(theta),theta};
	double[] myPose={x,y,theta};

	double angularVelocityDesired=speed/WHEEL_RADIUS_IN_M;

	//Set desired angular velocity
	robotVelocityController.setDesiredAngularVelocity(angularVelocityDesired,angularVelocityDesired);

	System.out.println("AVD: " + angularVelocityDesired);
	while (!comparePose(myPose, desiredPose, 0.1, 0.15)){
		myPose[0]=x;
		myPose[1]=y;
		myPose[2]=theta;
	}

	printPose();
	System.out.println("desiredPose: x: "+desiredPose[0]+" y:"+desiredPose[1]+" theta: "+desiredPose[2]);
	System.out.println(comparePose(myPose, desiredPose, 0.1, 0.15));
	System.out.println("We got there.");

	//Set angular velocity to 0
	robotVelocityController.setDesiredAngularVelocity(0,0);

	// End Student Code
	return ok;
  }

  /**
   * <p>Rotate at the specified speed for the specified angle.</p>
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
    double pose = {x,y,theta};
    double desiredPose = {x,y,theta + angle};
    
    double desiredAngularVelocity = (speed * DISTANCE_BETWEEN_WHEELS/2)/WHEEL_RADIUS_IN_M;

    double startingTime = totalTime;
    if(angle > 0)
	robotVelocityController.setDesiredAngularVelocity(-desiredAngularVelocity, desiredAngularVelocity);
    else
	robotVelocityController.setDesiredAngularVelocity(desiredAngularVelocity, -desiredAngularVelocity);

    while(!comparePose(pose,desiredPose, .1, .15)) {
	pose[0] = x;
	pose[1] = y;
	pose[2] = theta;
    }

    printPose();
    System.out.println("desiredPose: x: "+desiredPose[0]+" y:"+desiredPose[1]+" theta: "+desiredPose[2]);
    System.out.println(comparePose(myPose, desiredPose, 0.1, 0.15));
    System.out.println("We got there.");

    robotVelocityController.setDesiredAngularVelocity(0,0);
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
    robotVelocityController = (RobotVelocityControllerBalanced)vc;
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
   * <p>Check if pose1 is near pose 2 within certain tolerances</p>
   *
   * @param pose1 pose to check
   * @param pose2 pose to compare to
   * @param toleranceLinear linear tolerance in meters
   * @param toleranceAngular angular tolerance in radians
   **/

  public boolean comparePose(double [] pose1, double [] pose2, double toleranceLinear, double toleranceAngular){
	double distance = Math.pow(Math.pow(pose1[0] - pose2[0], 2) + Math.pow(pose1[1] - pose2[1], 2), 0.5);
	return (distance < toleranceLinear) && (Math.abs(pose1[2] - pose2[2]) < toleranceAngular);
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
    double rightDist = rightTicks / TICKS_PER_METER;
    double leftDist = leftTicks / TICKS_PER_METER;

    //Useful definitions
    double distDiff = rightDist - leftDist;
    double distAvg = (rightDist + leftDist) / 2;
    double dtheta = distDiff / DISTANCE_BETWEEN_WHEELS;
    double thetaTravel = theta + dtheta / 2;

    //update our current odometry
    double thetaNew = theta + dtheta;
    double xNew = x + Math.cos(thetaTravel) * distAvg;
    double yNew = y + Math.sin(thetaTravel) * distAvg;

    //keep thetaNew in range
    while (thetaNew >= 2 * Math.PI) {
    	thetaNew -= 2 * Math.PI;
    }
    while (thetaNew < 0) {
    	thetaNew += 2 * Math.PI;
    }

    //apply the new odometry
    x = xNew;
    y = yNew;
    theta = thetaNew;
  }
}
