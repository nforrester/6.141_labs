package LocalNavigation;

import java.util.concurrent.ArrayBlockingQueue;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import VisualServo.VisionGUI;

/**
 * 
 * @author previous TA's, prentice, vona, nforrest, raga, wyken, dgonz
 *
 */
public class LocalNavigation implements NodeMain, Runnable{
	private Node logNode;

	private static boolean RUN_SONAR_GUI = true;
	public SonarGUI gui;

	public static int STOP_ON_BUMP  = 0;
	public static int ALIGN_ON_BUMP = 1;
	public static int ALIGNING      = 2;
	public static int ALIGNED       = 3;
	private int state = STOP_ON_BUMP;

	protected boolean firstUpdate = true;

	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	private double sonarFront = 0.0;
	private double sonarBack = 0.0;

	private static double transSlow = 1;
	private static double rotSlow = 1;

	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarFrontSub;
	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarBackSub;
	private Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	private Publisher<MotionMsg> motorPub;
	private MotionMsg commandMotors;

	private Publisher<org.ros.message.std_msgs.String> statePub;
	private org.ros.message.std_msgs.String stateMsg;

	/**
	 * <p>Create a new LocalNavigation object.</p>
	 */
	public LocalNavigation() {

		setInitialParams();

		if (RUN_SONAR_GUI) {
			gui = new SonarGUI();
			gui.resetWorldToView(1.0,1.0,1.0);
			gui.funfun();
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
			sonarFront = message.range;
		} else {
			sensor = "Back";
			sonarBack = message.range;
		}
		logNode.getLog().info("SONAR: Sensor: " + sensor + " Range: " + message.range);
	}
	
	/**
	 * <p>Handle a BumpMsg.</p>
	 * 
	 * @param the message
	 */
	public void handleBump(org.ros.message.rss_msgs.BumpMsg message) {
		bumpLeft = message.left;
		bumpRight = message.right;
		logNode.getLog().info("BUMP: Left: " + message.left + " Right: " + message.right);
	}
	
	@Override
	public void run() {
		while (true) {
			if (RUN_SONAR_GUI) {
				//TODO: Change this approprately to update gui however necessary
				//gui.setVisionImage(dest.toArray(), width, height);
			}

			if (state == STOP_ON_BUMP) {
				if (bumpLeft || bumpRight) {
					commandMotors.rotationalVelocity = 0;
					commandMotors.translationalVelocity = 0;
				} else {
					commandMotors.rotationalVelocity = 0;
					commandMotors.translationalVelocity = transSlow;
				}
			} else if (state == ALIGN_ON_BUMP || state == ALIGNING) {
				if (state == ALIGN_ON_BUMP && (bumpLeft || bumpRight)) {
					changeState(ALIGNING);
				}
				if (state == ALIGNING) {
					if (bumpLeft && bumpRight) {
						commandMotors.rotationalVelocity = 0;
						commandMotors.translationalVelocity = 0;
						changeState(ALIGNED);
					} else if (bumpLeft) {
						commandMotors.rotationalVelocity = rotSlow;
						commandMotors.translationalVelocity = 0;
					} else if (bumpRight) {
						commandMotors.rotationalVelocity = -1 * rotSlow;
						commandMotors.translationalVelocity = 0;
					} else {
						commandMotors.rotationalVelocity = 0;
						commandMotors.translationalVelocity = transSlow;
					}
				}
			}

			// publish velocity messages to move the robot
			motorPub.publish(commandMotors);
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

		// initialize the ROS subscriptions to rss/Sonars
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					handleSonar(message);
				}
			});

		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
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

		// initialize the ROS publication to command/Motors
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		commandMotors = new MotionMsg();

		// initialize the ROS publication to rss/state
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		stateMsg = new org.ros.message.std_msgs.String();

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

	private void changeState(int newState) {
		state = newState;
		if (state == STOP_ON_BUMP) {
			stateMsg.data = "STOP_ON_BUMP";
		} else if (state == ALIGN_ON_BUMP) {
			stateMsg.data = "ALIGN_ON_BUMP";
		} else if (state == ALIGNING) {
			stateMsg.data = "ALIGNING";
		} else if (state == ALIGNED) {
			stateMsg.data = "ALIGNED";
		} else {
			stateMsg.data = "ERROR: unknown state";
		}
		statePub.publish(stateMsg);
	}
}
