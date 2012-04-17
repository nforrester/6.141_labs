/*
 * 
 */
package Grasping;

import org.ros.message.rss_msgs.ArmMsg;

public class ShoulderController extends JointController {

    public static final long MIN_PWM = 2300; 
    public static final long MAX_PWM = 600; 
    
    /////////////////////////////////////
    // theta = 0 ; PWM = 516
    // theta = pi/2 ; PWM = 1516
    ////////////////////////////////////
    
    public static final double slope = 0.0015707963267948967; 
    public static final double thetaIntercept = -0.8105309046261667; 
    
    /**
     * Gets the modified arm msg,
     * given the current arm msg
     * and the desired angle.
     * 
     * Note:- This method just modifies 
     * the part of the arm message 
     * corresponding to shoulder servo.
     * To publish the message, we need to
     * combine other servo msgs.
     * 
     * @param currentMsg
     *            the current arm msg
     * @param desiredAngle
     *            the desired angle
     * @return the modified arm msg
     */
   public ArmMsg getModifiedArmMsg(ArmMsg currentMsg, double desiredAngle) {
       double limittedAngle = this.limitAngle(desiredAngle);
       return new ArmMsg();
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
       return limitAngle((long)((angle - ShoulderController.thetaIntercept)/ShoulderController.slope));
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
       return ShoulderController.slope*limitAngle(pwmValue) + ShoulderController.thetaIntercept;
   }
   
   /**
     * Limits the PWM
     * depending on the constraints
     * of shoulder servo.
     * 
     * @param angle
     *            the input angle
     * @return the limitted angle
     */
   private static long limitAngle(long pwmAngle) {
       long copyAngle = pwmAngle;
       if(copyAngle > ShoulderController.MAX_PWM) copyAngle = ShoulderController.MAX_PWM;
       else if(copyAngle < ShoulderController.MIN_PWM) copyAngle = ShoulderController.MIN_PWM;
       return copyAngle;
   }
    
}
