package MotorControl;

/**
 * <p>Closed-loop integral wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerI extends WheelVelocityController {

  /**
   * <p>The result of the previous control step.</p>
   **/
  protected double lastResult = 0;
  
  /**
   * <p>The integral of the error.</p>
   */
  protected double errorIntegral = 0;
  
  protected static final double INTEGRAL_GAIN = 1.2;

  protected static final double PROPOTIONAL_GAIN_BIG = 10;

  protected static final double PROPOTIONAL_GAIN_SMALL = 5;
  
  /**
   * {@inheritDoc}
   *
   * <p>This impl implements closed-loop integral control.</p>
   **/
  public double controlStep() {

    double result = 0;
    // Start Student Code
    double error = desiredAngularVelocity - currentAngularVelocity;
    errorIntegral += error*sampleTime;
    result = PROPOTIONAL_GAIN_BIG*error + INTEGRAL_GAIN*errorIntegral;
    // End Student Code

    if (result > MAX_PWM)
      result = MAX_PWM;

    if (result < -MAX_PWM)
      result = -MAX_PWM;

    lastResult = result;

    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "I".</p>
   **/
  public String getName() {
    return "I";
  }
}
