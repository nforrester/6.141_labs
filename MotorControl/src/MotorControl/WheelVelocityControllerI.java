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
  
  //protected static final double PROPROTIONAL_GAIN = MAX_PWM / MAX_ANGULAR_VELOCITY;
  protected static final double PROPROTIONAL_GAIN = 0.45 * 60;

  protected static final double INTEGRAL_GAIN = 1.2 * PROPROTIONAL_GAIN / 0.24;
  
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
    result = gain * PROPROTIONAL_GAIN*error + INTEGRAL_GAIN*errorIntegral;
    System.out.println("-----------------------------");
    System.out.println(MAX_PWM);
    System.out.println(MAX_ANGULAR_VELOCITY);
    System.out.println(PROPROTIONAL_GAIN);
    System.out.println(INTEGRAL_GAIN);
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
