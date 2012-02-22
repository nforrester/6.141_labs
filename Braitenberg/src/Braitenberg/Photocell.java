package Braitenberg;

import MotorControlSolution.*;
//import MotorControl.*;

/**
 * <p>One light sensor.</p>
 **/
public class Photocell {

  /**
   * <p>The robot containing this light sensor.</p>
   **/
  protected RobotBase robot;

  /**
   * <p>The input port to which the sensor is connected.</p>
   *
   * <p>See {@link RobotBase#analogRead}.</p>
   **/
  protected int port;

  /**
   * <p>The calibration offset.</p>
   **/
  protected double offset = 0.0;

  /**
   * <p>The calibration scale.</p>
   **/
  protected double scale = 1.0;

  /**
   * <p>Create a new photocell object.</p>
   *
   * @param robot the robot containing this photocell
   * @param port the input port to which the sensor is connected (see {@link
   * RobotBase#analogRead})
   **/
  public Photocell(RobotBase robot, int port) {
    this.robot = robot;
    this.port = port;
  }

  /**
   * <p>Get the raw value of the sensor.</p>
   *
   * <p>Reads {@link #port} of {@link #robot}.</p>
   *
   * @return the raw value of the sensor
   **/
  protected double getRawValue() {
    return robot.analogRead(port);
  }

  /**
   * <p>Get the calibrated value of the sensor.</p>
   *
   * @return the calibrated value of the sensor, in the range [0.0, 100.0]
   **/
  public double getValue() {

    double val = 0.0;

    // Begin Student Code
    // End Student Code

    if (val > 100.0)
      val = 100.0;
    if (val < 0.0)
      val = 0.0;

    return val;
  }

  /**
   * <p>Calibrate the sensor.</p>
   *
   * <p>Blocks while taking readings, then sets {@link #offset} and {@link
   * #scale}.</p>
   **/
  public void calibrate() {
    // Begin Student Code
    // End Student Code
  }
}
