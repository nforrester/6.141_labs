package MotorControl;

/**
 * <p>Feed-forward wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerFF extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple feed-forward control.</p>
   **/
  public double controlStep() {
    double result = 0;
    result = MAX_PWM * desiredAngularVelocity / MAX_ANGULAR_VELOCITY;
    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "FF".</p>
   **/
  public String getName() {
    return "FF";
  }
}
