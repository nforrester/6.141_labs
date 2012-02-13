package MotorControl;

/**
 * <p>Generic base class for all velocity controllers.</p>
 *
 * @author vona
 * @author prentice
 **/
public abstract class VelocityController {

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
   * <p>Subclasses should override this to accept the feedback they need, and
   * chain to this impl to save the sample time.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   **/
  public void update(double time) {
    sampleTime = time;
  }

  /**
   * <p>Return a short name for this velocity controller.</p>
   *
   * @return a short name for this velocity controller
   **/
  public abstract String getName();
}
