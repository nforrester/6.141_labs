package MotorControl;

/**
 * <p>Whole-robot velocity controller.</p>
 *
 * <p>Default impl just passes through to two {@link WheelVelocityController}s,
 * which are passed to the constructor.</p>
 *
 * <p>Subclasses may do more interesting things.</p>
 **/
public class RobotVelocityController extends VelocityController {

  /**
   * <p>The two wheel velocity controllers.</p>
   **/
  protected WheelVelocityController[] wheelVelocityController =
  new WheelVelocityController[2];

  /**
   * <p>Construct a new whole-robot velocity controller.</p>
   *
   * @param leftWheelVelocityController the left-wheel controller
   * @param rightWheelVelocityController the right-wheel controller
   **/
  public RobotVelocityController(
    WheelVelocityController leftWheelVelocityController,
    WheelVelocityController rightWheelVelocityController) {
    wheelVelocityController[RobotBase.LEFT] = leftWheelVelocityController;
    wheelVelocityController[RobotBase.RIGHT] = rightWheelVelocityController;
  }

  /**
   * <p>Get the velocity controller for a wheel.</p>
   *
   * @param wheel {@link RobotBase#LEFT} or {@link RobotBase#RIGHT}
   * @return the velocity controller for wheel
   **/
  public WheelVelocityController getWheelVelocityController(int wheel) {
    return wheelVelocityController[wheel];
  }

  /**
   * <p>Compute a whole-robot velocity control step.</p>
   *
   * <p>Default impl just asks each {@link #wheelVelocityController} to do its
   * {@link WheelVelocityController#controlStep}.  Subclasses may override this
   * to do more interesting things.</p>
   *
   * @param controlOutput the left and right control outputs (i.e. pwm
   * commands) are written here on return, positive means
   * corresp side of robot rolled forward
   **/
  public void controlStep(double[] controlOutput) {
    controlOutput[RobotBase.LEFT] =
      wheelVelocityController[RobotBase.LEFT].controlStep();
    controlOutput[RobotBase.RIGHT] =
      -wheelVelocityController[RobotBase.RIGHT].controlStep();
  }
    
  /**
   * <p>Set the desired angular velocity for both wheels.</p>
   *
   * <p>Default impl just {@link
   * WheelVelocityController#setDesiredAngularVelocity} on each {@link
   * #wheelVelocityController}.  Subclasses may override this to do more
   * interesting things.</p>
   *
   * @param left the left desired angular velocity in rad/s, positive means
   * corresp side of robot rolled forward
   * @param right the right desired angular velocity in rad/s, positive means
   * corresp side of robot rolled forward
   **/
  public void setDesiredAngularVelocity(double left, double right) {
    wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(left);
    wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(-right);
  }

  /**
   * <p>Set the desired angular velocity for one wheel.</p>
   *
   * <p>The desired angular velocity for the other wheel is not changed.</p>
   *
   * <p>Default impl just {@link
   * WheelVelocityController#setDesiredAngularVelocity} on {@link
   * #wheelVelocityController}[wheel].  Subclasses may override this to do more
   * interesting things.</p>
   *
   * @param wheel {@link RobotBase#LEFT} or {@link RobotBase#RIGHT}
   * @param velocity the desired angular velocity in rad/s, positive means
   * corresp side of robot rolled forward
   **/
  public void setDesiredAngularVelocity(int wheel, double velocity) {
    wheelVelocityController[wheel].
      setDesiredAngularVelocity(velocity*
                                ((wheel == RobotBase.RIGHT) ? -1.0 : 1.0));
  }

  /**
   * <p>Get the most-recently set desired angular velocity for a wheel.</p>
   *
   * <p>Default impl just {@link
   * WheelVelocityController#getDesiredAngularVelocity} on {@link
   * #wheelVelocityController}[wheel].  Subclasses may override this to do more
   * interesting things.</p>
   *
   * @param wheel {@link RobotBase#LEFT} or {@link RobotBase#RIGHT}
   * @param return the most-recently set desired angular velocity in rad/s for
   * wheel, positive means wheel side of robot rolled forward
   **/
  public double getDesiredAngularVelocity(int wheel) {
    return
      wheelVelocityController[wheel].getDesiredAngularVelocity()*
      ((wheel == RobotBase.RIGHT) ? -1.0 : 1.0);
  }

  /**
   * <p>Compute the angular velocity feedback of a wheel in rad/s.</p>
   *
   * <p>Default impl just {@link
   * WheelVelocityController#computeAngularVelocity}. Subclasses may override
   * this to do more interesting things.</p>
   *
   * @param wheel {@link RobotBase#LEFT} or {@link RobotBase#RIGHT}
   * @return the angular velocity feedback of wheel in rad/s, positive means
   * wheel side of robot rolled forward
   **/
  public double computeAngularVelocity(int wheel) {
    return
      wheelVelocityController[wheel].computeAngularVelocity()*
      ((wheel == RobotBase.RIGHT) ? -1.0 : 1.0);
  }

  /**
   * <p>Update feedback and sample time.</p>
   *
   * <p>Default impl calls {@link WheelVelocityController#update} for each
   * wheel.  Subclasses may override this to do something different.</p>
   *
   * @param time the time in seconds since the last update
   * @param leftTicks left encoder ticks since last update, positive means
   * corresp side of robot rolled forward
   * @param rightTicks right encoder ticks since last update, positive means
   * corresp side of robot robot rolled forward
   **/
  public void update(double time, double leftTicks, double rightTicks) {
    super.update(time);
    wheelVelocityController[RobotBase.LEFT].update(time, leftTicks);
    wheelVelocityController[RobotBase.RIGHT].update(time, -rightTicks);
  }

  /**
   * {@inheritDoc}
   *
   * <p>Default impl returns "pass-through".</p>
   **/
  public String getName() {
    return "pass-through";
  }
}
