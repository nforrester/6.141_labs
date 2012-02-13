package MotorControl;

/**
 * <p>Open loop wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerPWM extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple open-loop control.</p>
   **/
  public double controlStep() {
    double result = 0;
    // Start Student Code
    // End Student Code
    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "PWM".</p>
   **/
  public String getName() {
    return "PWM";
  }
}
