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

  /**
   * <p>The robot.</p>
   **/
  protected OdometryRobot robot;

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
  }
}
