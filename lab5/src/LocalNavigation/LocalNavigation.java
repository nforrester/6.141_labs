package LocalNavigation;

import java.util.concurrent.ArrayBlockingQueue;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
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
	private int state = ALIGNED;

	protected boolean firstUpdate = true;

	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	private static double transSlow = 1;
	private static double rotSlow = 1;

	/* Frames of reference:
	 *
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 * Odometry frame: Whatever stupid frame of reference the odometry module uses
	 */

	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry

	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry

	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry

	// transforms between sonar and robot frames
	private static final Mat sonarToRobotRot = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.multiply(Mat.translation( 0.1016, 0.2286), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.multiply(Mat.translation(-0.2540, 0.2286), sonarToRobotRot);

	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarFrontSub;
	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarBackSub;
	private Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	private Publisher<MotionMsg> motorPub;
	private Publisher<GUIPointMsg> pointPub;
	private MotionMsg commandMotors;
	private GUIPointMsg pointPlot;

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

		Mat sonarToWorld;

		if (message.isFront) {
			sensor = "Front";
			sonarToWorld = Mat.multiply(robotToWorld, sonarFrontToRobot);
			pointPlot.shape = 0;
			
		} else {
			sensor = "Back";
			sonarToWorld  = Mat.multiply(robotToWorld, sonarBackToRobot);
			pointPlot.shape = 1;
		}

		double[] echoWorld = Mat.decodePose(Mat.multiply(sonarToWorld, Mat.encodePose(message.range, 0, 0)));
		pointPlot.x = echoWorld[0];
		pointPlot.y = echoWorld[1];
		pointPub.publish(pointPlot);
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
	
	/**
	 * <p>Handle an OdometryMsg.</p>
	 * 
	 * @param the message
	 */
	public void handleOdometry(org.ros.message.rss_msgs.OdometryMsg message) {
		if ( firstUpdate ) {
			firstUpdate = false;

			odoToWorld = Mat.multiply(Mat.rotation(-message.theta), Mat.translation(-message.x, -message.y));

			if (RUN_SONAR_GUI) {
				gui.resetWorldToView(0, 0);
			}
		}

		double[] robotPose = Mat.decodePose(Mat.multiply(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		robotToWorld = Mat.multiply(Mat.translation(x, y), Mat.rotation(theta));

		if (RUN_SONAR_GUI) {
			gui.setRobotPose(x, y, theta);
		}
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
					handleOdometry(message);
				}
			});

		// initialize the ROS publication to command/Motors
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		commandMotors = new MotionMsg();
		
		// initialize the ROS publication to graph points
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		pointPlot=new GUIPointMsg();

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
