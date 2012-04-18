package Grasping;

import org.ros.message.rss_msgs.ArmMsg;
import org.ros.node.Node;

public class WristController extends JointController {

    public WristController(Node node) {
		super(node);
	}

	public static final long MIN_PWM = 435;
    public static final long MAX_PWM = 2101; 
    
    /*
    /////////////////////////////////////
    // theta = 0 ; PWM = 1210
    // theta = pi/2 ; PWM = 2075
    ////////////////////////////////////
    
    public static final double slope = 0.0018159495107455451; 
    public static final double thetaIntercept = -2.1972989080021095; 
    */
    
    static final double theta1 = 0;
    static final double theta2 = -1 * Math.PI / 2;

    static final double pwm1 = 1250;
    static final double pwm2 = 2100;

    public static final double slope = (theta2 - theta1) / (pwm2 - pwm1);
    public static final double thetaIntercept = theta1 - slope * pwm1;

    /**
     * Gets the modified arm msg,
     * given the current arm msg
     * and the desired angle.
     * 
     * Note:- This method just modifies 
     * the part of the arm message 
     * corresponding to wrist servo.
     * To publish the message, we need to
     * combine other servo msgs.
     * 
     * @param currentMsg
     *            the current arm msg
     * @param desiredAngle
     *            the desired angle
     * @return the modified arm msg
     */
   public static ArmMsg getArmMsgForAngle(double desiredAngle) {
	   long[] currentArray = {0,0,0,0,0,0,0,0};
       currentArray[4] = getPWMEquivalent(desiredAngle);
       ArmMsg returnMessage = new ArmMsg();
       returnMessage.pwms = currentArray;
       return returnMessage;
   }
   
   /**
    * Gets the pWM equivalent
    * of a given angle.
    * 
    * @param angle
    *            the angle
    * @return the pWM equivalent
    */
  public static long getPWMEquivalent(double angle) {
      return limitAngle((long)((angle - WristController.thetaIntercept)/WristController.slope));
  }
  
  /**
    * Gets the limitted angle 
    * equivalent for a given 
    * PWM value.
    * 
    * @param pwmValue
    *            the pwm value
    * @return the angle equivalent
    */
  public static double getAngleEquivalent(long pwmValue) {
      return WristController.slope*limitAngle(pwmValue) + WristController.thetaIntercept;
  }
  
  /**
    * Limits the PWM
    * depending on the constraints
    * of wrist servo.
    * 
    * @param angle
    *            the input angle
    * @return the limitted angle
    */
  private static long limitAngle(long pwmAngle) {
      long copyAngle = pwmAngle;
      if(copyAngle > WristController.MAX_PWM) copyAngle = WristController.MAX_PWM;
      else if(copyAngle < WristController.MIN_PWM) copyAngle = WristController.MIN_PWM;
      return copyAngle;
  }
   
    
}
