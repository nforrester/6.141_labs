
package Grasping;

import org.ros.message.rss_msgs.ArmMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

public class JointController implements NodeMain {
        
    public static final double servoCommandAngleTreshold = 0.1;
     
    	Publisher<ArmMsg> armPublisher;
	
	
    	/**
    	 * Commands the servo to move
    	 * by a treshold step towards
    	 * the goal pose.
    	 * 
    	 * @param currentMessage of the Arm
    	 * 
    	 * @param desiredPose
    	 */
		public void commandServos(ArmMsg currentMessage, double[] desiredAngles) {
				// 0 shoulder
	    	   	// 1 wrist
	    	   	// 2 gripper
			
			Grasping.targetAngles = new double[]{desiredAngles[0],desiredAngles[1],desiredAngles[2]};
			
			//get the actual angles
			double currentShoulderAngle = ShoulderController.getAngleEquivalent(currentMessage.pwms[5]);
			double currentWristAngle = WristController.getAngleEquivalent(currentMessage.pwms[4]);
			double currentGripperAngle = GripperController.getAngleEquivalent(currentMessage.pwms[2]);
			
			//limit the angles
			double shoulderError = currentShoulderAngle + limitChangeCommandAngle(currentShoulderAngle - desiredAngles[0]);							
			double wristError =	currentWristAngle + limitChangeCommandAngle(currentWristAngle - desiredAngles[1]);								
			double gripperError =currentGripperAngle + limitChangeCommandAngle(currentGripperAngle - desiredAngles[2]);

			ArmMsg publishMsg = getEquivalentArmMsg(new double[] {shoulderError,wristError,gripperError});
			
			//publish the message made
			this.armPublisher.publish(publishMsg);
			
		}
       
       
       
		
		
		
		/**
		 * Limits the given angle
		 * within the given threshold
		 * number of radians
		 * 
		 * @param angle
		 * @return the limitted angle
		 */
		private static double limitChangeCommandAngle(double angle){
			double copy = angle;
			if(copy > JointController.servoCommandAngleTreshold){
				copy = JointController.servoCommandAngleTreshold;
			}
			else if(copy < -JointController.servoCommandAngleTreshold){
				copy = -JointController.servoCommandAngleTreshold;
			}
			return copy;
		}
       
       
       
       /**
        * Returns an arm message after
        * decoding the given angles.
        * 
        * @param desiredAngles
        * @return Armmsg corresponding
        * to the desired angles
        */
       private static ArmMsg getEquivalentArmMsg(double[] desiredAngles){
    	   // 0 shoulder
    	   // 1 wrist
    	   // 2 gripper
    	   
    	   ArmMsg shoulderMsg = ShoulderController.getArmMsgForAngle(desiredAngles[0]);
    	   ArmMsg WristMsg = WristController.getArmMsgForAngle(desiredAngles[1]);
    	   ArmMsg GripperMsg = GripperController.getArmMsgForAngle(desiredAngles[2]);
    	   
    	   ArmMsg returnMessage = new ArmMsg();
    	   returnMessage.pwms = new long[] {0,0,GripperMsg.pwms[2],0,WristMsg.pwms[4],shoulderMsg.pwms[5],0,0,0};
    	   return returnMessage;
       }






	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(Node node) {
		armPublisher = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
		
	}



	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("team6/JointController");
	}
       
       
}
