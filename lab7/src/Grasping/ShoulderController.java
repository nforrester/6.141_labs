package Grasping;

import org.ros.message.rss_msgs.ArmMsg;

public class ShoulderController extends JointController {

    public static final long MIN_PWM = 2300; 
    public static final long MAX_PWM = 600; 
    
    public static final double slope = 0; //to be set
    public static final double thetaIntercept = 0; //to be set
    
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
     * Limits the angle
     * depending on the constraints
     * of shoulder servo.
     * 
     * @param angle
     *            the input angle
     * @return the limitted angle
     */
   public double limitAngle(double angle) {
       // to be written
       return 0;
   }
    
}
