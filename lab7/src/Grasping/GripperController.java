package Grasping;

import org.ros.message.rss_msgs.ArmMsg;

public class GripperController extends JointController {

    public static final long MIN_PWM = 450; 
    public static final long MAX_PWM = 1650; 
    
    /////////////////////////////////////
    // theta = 0 ; PWM = 452
    // theta = pi/2 ; PWM = 1645
    ////////////////////////////////////
    
    
    public static final double slope = 0.0013166775580845738; 
    public static final double thetaIntercept = -0.5951382562542273; 
    
    /**
     * Gets the modified arm msg,
     * given the current arm msg
     * and the desired angle.
     * 
     * Note:- This method just modifies 
     * the part of the arm message 
     * corresponding to gripper servo.
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
       currentArray[2] = getPWMEquivalent(desiredAngle);
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
      return limitAngle((long)((angle - GripperController.thetaIntercept)/GripperController.slope));
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
      return GripperController.slope*limitAngle(pwmValue) + GripperController.thetaIntercept;
  }
  
  /**
    * Limits the PWM
    * depending on the constraints
    * of gripper servo.
    * 
    * @param angle
    *            the input angle
    * @return the limitted angle
    */
  private static long limitAngle(long pwmAngle) {
      long copyAngle = pwmAngle;
      if(copyAngle > GripperController.MAX_PWM) copyAngle = GripperController.MAX_PWM;
      else if(copyAngle < GripperController.MIN_PWM) copyAngle = GripperController.MIN_PWM;
      return copyAngle;
  }



  
}
