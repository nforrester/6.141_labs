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
  
  /**
   * <p>The coefficent of the IIR error integrator.</p>
   */
  protected static final double INTEGRAL_COEFF = 0.9;
  
  protected static final double INTEGRAL_GAIN = 1.5;

  protected static final double PROPOTIONAL_GAIN = MAX_PWM / MAX_ANGULAR_VELOCITY;
  
  /**
   * {@inheritDoc}
   *
   * <p>This impl implements closed-loop integral control.</p>
   **/
  public double controlStep() {

    double result = 0;
    // Start Student Code
    double error = desiredAngularVelocity - currentAngularVelocity;
    errorIntegral = errorIntegral * (1 - ((1 - INTEGRAL_COEFF) * sampleTime)) + error * sampleTime;
    result = INTEGRAL_GAIN * errorIntegral;
    result += PROPOTIONAL_GAIN * error;
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
