package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>Robot with an odometry-based position controller.</p>
 **/
public class OdometryRobot extends RobotBase {

  /**
   * <p>Whole-robot position control; needs to be externally set.</p>
   **/
  protected RobotPositionController robotPositionController;

  /**
   * <p>Set the whole-robot position controller.</p>
   *
   * @param vc the whole-robot position controller
   **/
  public void setRobotPositionController(RobotPositionController pc) {

    robotPositionController = pc;

    if (pc != null)
      pc.setRobotVelocityController(robotVelocityController);
  }

  /**
   * <p>Get the whole-robot position controller.</p>
   *
   * @return the whole-robot position controller
   **/
  public RobotPositionController getRobotPositionController() {
    return robotPositionController;
  }

  /**
   * {@inheritDoc}
   *
   * <p>Extends superclass impl to tell {@link #robotPositionController} about
   * vc.</p>
   **/
  public void setRobotVelocityController(RobotVelocityController vc) {

    super.setRobotVelocityController(vc);

    if (robotPositionController != null)
      robotPositionController.setRobotVelocityController(vc);
  }

  /**
   * {@inheritDoc}
   *
   * <p>Extends superclass impl to first update {@link
   * #robotPositionController}.</p>
   **/
  protected void updateAndControl(double sampleTime, int encL, int encR) {

    //tell the position controller the new feedback & sample time
    robotPositionController.update(sampleTime, encL, encR);
   
    //update the position control loop, if any
    robotPositionController.controlStep();

    super.updateAndControl(sampleTime, encL, encR);
  }
}
