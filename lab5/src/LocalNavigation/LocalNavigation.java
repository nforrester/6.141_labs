package LocalNavigation;

import java.util.concurrent.ArrayBlockingQueue;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import MotorControlSolution.*;

/**
 * 
 * @author previous TA's, prentice, vona, nforrest, raga, wyken, dgonz
 *
 */
public class VisualServo implements NodeMain, Runnable{

	protected RobotVelocityController robotVelocityController;
	private MotionMsg commandMotors;
	private Publisher<MotionMsg> k;

	private Node logNode;

	private static boolean RUN_SONAR_GUI = true;
	private SonarGUI gui;

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarFrontSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarBackSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	/**
	 * <p>Create a new LocalNavigation object.</p>
	 */
	public LocalNavigation() {

		setInitialParams();

		if (RUN_SONAR_GUI) {
			gui = new SonarGUI();
		}
	}
	
	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a SonarMsg.</p>
	 * 
	 * @param the message
	 */
	public void handleSonar(org.ros.message.rss_msgs.SonarMsg message) {
		String sensor = new String();
		if (message.isFront) {
			sensor = "Front";
		} else {
			sensor = "Back";
		}
		logNode.getLog().info("SONAR: Sensor: " + sensor + " Range: " message.range);
	}
	
	/**
	 * <p>Handle a BumpMsg.</p>
	 * 
	 * @param the message
	 */
	public void handleBump(org.ros.message.rss_msgs.BumpMsg message) {
		logNode.getLog().info("BUMP: Left: " + message.left + " Right: " message.right);
	}
	
	@Override
	public void run() {
		while (true) {
			if (RUN_SONAR_GUI) {
				//TODO: Change this approprately to update gui however necessary
				gui.setVisionImage(dest.toArray(), width, height);
			}

			// publish velocity messages to move the robot
			commandMotors.rotationalVelocity = 0; //TODO
			commandMotors.translationalVelocity = 0; //TODO

			k.publish(commandMotors);
		}
	}

	/**
	 * <p>
	 * Run the LocalNavigation process
	 * </p>
	 * 
	 * @param optional
	 *            command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		logNode = node;

		// initialize the ROS publication to command/Motors
		k = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		commandMotors = new MotionMsg();

		// initialize the ROS subscriptions to rss/Sonars
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					handleSonar(message);
				}
			});

		sonarBackSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		sonarBackSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		sonarBackSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
					handleBump(message);
				}
			});

		// initialize the ROS subscription to rss/odometry
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
					if ( firstUpdate ) {
						firstUpdate = false;
						if (RUN_SONAR_GUI) {
							gui.resetWorldToView(message.x, message.y);
						}
					}
					if (RUN_SONAR_GUI) {
						gui.setRobotPose(message.x, message.y, message.theta);
					}
				}
			});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	@Override
	public void onShutdown(Node node){
		if(node != null){
			node.shutdown();
		} 
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/localnavigation");
	}
}
