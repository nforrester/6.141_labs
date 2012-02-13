package MotorControl;

/**
 * <p>Whole-robot velocity controller with side-to side balance integrator.</p>
 *
 * <p>Uses {@link WheelVelocityControllerFF} for the low-level per-side
 * control, and internally maintains an integrator to avoid inter-side
 * drift.</p>
 *
 * @author vona
 **/
public class RobotVelocityControllerBalanced extends RobotVelocityController {

  /**
   * <p>The desired angular velocity of each wheel in rad/s, positive means
   * robot moves forward.</p>
   **/
  protected double[] desiredAngularVelocity = new double[2];

  /**
   * <p>The average desired angular velocity of both wheels in rad/s, positive
   * means robot moves forward.</p>
   **/
  protected double desiredAngularVelocityAvg;

  /**
   * <p>Difference between each side's desired angular velocity in rad/s,
   * positive if left moves forward faster than right.</p>
   **/
  protected double desiredAngularVelocityDiff;

  /**
   * <p>Total integrated actual angle differential in radians, positive means
   * left wheel leads right.</p>
   **/
  protected double actualDiffAngle;

  /**
   * <p>Desired integrated angle differential in radians, positive means left
   * wheel leads right.</p>
   **/
  protected double desiredDiffAngle;

  /**
   * <p>Debug counter.</p>
   **/
  protected int updateDbg = 0;

  /**
   * <p>Construct a new whole-robot balanced velocity controller.</p>
   *
   * @param leftWheelVelocityController the left-wheel controller
   * @param rightWheelVelocityController the right-wheel controller
   **/
  public RobotVelocityControllerBalanced() {
    super(new WheelVelocityControllerI(), new WheelVelocityControllerI());
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements proportional feedback control on each side and
   * integral feedback control between the sides.</p>
   **/
  public void controlStep(double[] controlOutput) {

    //difference between desiredDiffAngle and actualDiffAngle in radians
    //will be positive if right is leading
    double angularDiffError = 0.0;

    //left and right desired velocities in rad/s, positive means corresp side
    //moves forward, after incorporating balance
    double ldv = 0.0, rdv = 0.0;

    // Begin Student Code




    // End Student Code

//    if ((updateDbg++ % 10) == 0) {
//      System.err.println("des avg vel: " + desiredAngularVelocityAvg);
//      System.err.println("des diff vel: " + desiredAngularVelocityDiff);
//      System.err.println("des diff angle: " + desiredDiffAngle);
//      System.err.println("act diff angle: " + actualDiffAngle);
//      System.err.println("err angle: " + angularDiffError);
//      System.err.println("ldv: " + ldv + "; rdv: " + rdv);
//    }

    wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(ldv);
    wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(-rdv);

    controlOutput[RobotBase.LEFT] =
      wheelVelocityController[RobotBase.LEFT].controlStep();
    controlOutput[RobotBase.RIGHT] =
      -wheelVelocityController[RobotBase.RIGHT].controlStep();
  }
    
  /**
   * {@inheritDoc}
   *
   * <p>This impl stores the settings in {@link #desiredAngularVelocity},
   * {@link #desiredAngularVelocityAvg}, and {@link
   * #desiredAngularVelocityDiff}.</p>
   **/
  public void setDesiredAngularVelocity(double left, double right) {

    desiredAngularVelocity[RobotBase.LEFT] = left;
    desiredAngularVelocity[RobotBase.RIGHT] = right;

    desiredAngularVelocityAvg = (left + right)/2.0;
    desiredAngularVelocityDiff = left-right;

//    System.err.println("des avg: " + desiredAngularVelocityAvg + 
//                       "; des diff: " + desiredAngularVelocityDiff);
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl covers {@link #setDesiredAngularVelocity(double,
   * double)}.</p>
   **/
  public void setDesiredAngularVelocity(int wheel, double velocity) {

    double l = desiredAngularVelocity[RobotBase.LEFT];
    double r = desiredAngularVelocity[RobotBase.RIGHT];

    if (wheel == RobotBase.LEFT)
      l = velocity;

    if (wheel == RobotBase.RIGHT)
      r = velocity;

    setDesiredAngularVelocity(l, r);
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns {@link #desiredAngularVelocity}[wheel].</p>
   **/
  public double getDesiredAngularVelocity(int wheel) {
    return desiredAngularVelocity[wheel];
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl chains to superclass impl, then updates {@link
   * #actualDiffAngle} and {@link #desiredDiffAngle}.</p>
   **/
  public void update(double time, double leftTicks, double rightTicks) {
    super.update(time, leftTicks, rightTicks);

    actualDiffAngle +=
      leftTicks *
      wheelVelocityController[RobotBase.LEFT].computeRadiansPerTick();

    actualDiffAngle -=
      rightTicks *
      wheelVelocityController[RobotBase.RIGHT].computeRadiansPerTick();

    desiredDiffAngle += desiredAngularVelocityDiff*time;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "balanced".</p>
   **/
  public String getName() {
    return "balanced";
  }
}
