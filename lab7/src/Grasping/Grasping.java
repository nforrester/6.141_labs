/*
 * 
 */
package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

public class Grasping implements NodeMain{

	public static double[] targetAngles = {0.0,0.0,0.0};
	private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;

	public static void main(String[] args){

	}





	public class ArmListener implements MessageListener<ArmMsg> {

		@Override public void onNewMessage(ArmMsg currentMessage) {
			double currentShoulderAngle = ShoulderController.getAngleEquivalent(currentMessage.pwms[5]);
			double currentWristAngle = WristController.getAngleEquivalent(currentMessage.pwms[4]);
			double currentGripperAngle = GripperController.getAngleEquivalent(currentMessage.pwms[2]);
			
			if((currentShoulderAngle != targetAngles[0]) || (currentWristAngle != targetAngles[1])
					|| (currentGripperAngle != targetAngles[2])){
				
				new JointController().commandServos(currentMessage, targetAngles);
			}
		}
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
		armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
		armSub.addMessageListener(new ArmListener(...));

	}




	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}
}
