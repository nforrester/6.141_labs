package LocalNavigation;

import java.util.concurrent.ArrayBlockingQueue;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.ColorMsg;
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

	private static boolean RUN_SONAR_GUI = false;
	public SonarGUI gui;

	public static int STOP_ON_BUMP    = 0;
	public static int ALIGN_ON_BUMP   = 1;
	public static int ALIGNING        = 2;
	public static int ALIGNED         = 3;
	public static int SPIN_ONCE_START = 4;
	public static int SPIN_ONCE       = 5;
	public static int SPIN_ONCE_STOP  = 6;
	public static int MANUAL_MODE     = 7;
	private int state = MANUAL_MODE;

	protected boolean firstUpdate = true;

	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	private static double transSlow = 0.05;
	private static double rotSlow = 0.05;
	private static double rotFast = 0.2;

	/* Frames of reference:
	 *
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 * Odometry frame: Whatever stupid frame of reference the odometry module uses, which the SonarGUI also uses
	 */

	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry

	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry
	private Mat worldToOdo; // initialized in handleOdometry

	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry

	// transforms between sonar and robot frames
	private static final Mat sonarToRobotRot = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.mul(Mat.translation( 0.1016, 0.2286), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.mul(Mat.translation(-0.2540, 0.2286), sonarToRobotRot);

	private LeastSquareLine lsq;

	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarFrontSub;
	private Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarBackSub;
	private Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;


	private Publisher<MotionMsg> motorPub;
	private MotionMsg commandMotors;

	private Publisher<GUIPointMsg> pointPub;
	private GUIPointMsg pointPlot;
	private ColorMsg pointPlotColor;

	private Publisher<GUILineMsg> linePub;
	private GUILineMsg linePlot;
	private ColorMsg linePlotColor;

	private Publisher<org.ros.message.std_msgs.String> statePub;
	private org.ros.message.std_msgs.String stateMsg;

	/**
	 * <p>Create a new LocalNavigation object.</p>
	 */
	public LocalNavigation() {

		setInitialParams();

		lsq = new LeastSquareLine();

		if (RUN_SONAR_GUI) {
			gui = new SonarGUI();
			gui.resetWorldToView(1.0,1.0,1.0);
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

		Mat sonarToRobot;

		if (message.isFront) {
			sensor = "Front";
			sonarToRobot = sonarFrontToRobot;
			pointPlot.shape = 0;
			
		} else {
			sensor = "Back";
			sonarToRobot = sonarBackToRobot;
			pointPlot.shape = 1;
		}

		if (!firstUpdate) {
			Mat echoSonar = Mat.encodePose(message.range, 0, 0);

			Mat echoWorld = Mat.mul(robotToWorld, sonarToRobot, echoSonar);
			Mat echoOdo = Mat.mul(worldToOdo, echoWorld);

			double[] echoWorldL = Mat.decodePose(echoWorld);
			double[] echoOdoL = Mat.decodePose(echoOdo);

			if (message.range > 1.0) {
				pointPlotColor.r = 0;
				pointPlotColor.g = 0;
				pointPlotColor.b = 255;
			} else {
				pointPlotColor.r = 255;
				pointPlotColor.g = 0;
				pointPlotColor.b = 0;

				lsq.addPoint(echoWorldL[0], echoWorldL[1]);
				double[] line = lsq.getLine();
				if (line.length > 0) {
					linePlot.lineA = line[0];
					linePlot.lineB = line[1];
					linePlot.lineC = line[2];
					linePlotColor.r = 0;
					linePlotColor.g = 130;
					linePlotColor.b = 0;
					linePlot.color = linePlotColor;
					linePub.publish(linePlot);
				}
			}
			pointPlot.color = pointPlotColor;
			pointPlot.x = echoOdoL[0];
			pointPlot.y = echoOdoL[1];
			pointPub.publish(pointPlot);
		}
		logNode.getLog().info("SONAR: Sensor: " + sensor + " Range: " + message.range);
		motorUpdate();
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
		motorUpdate();
	}
	
	/**
	 * <p>Handle an OdometryMsg.</p>
	 * 
	 * @param the message
	 */
	public void handleOdometry(org.ros.message.rss_msgs.OdometryMsg message) {
		if ( firstUpdate ) {
			odoToWorld = Mat.mul(Mat.rotation(-message.theta), Mat.translation(-message.x, -message.y));
			worldToOdo = Mat.inverse(odoToWorld);

			if (RUN_SONAR_GUI) {
				gui.resetWorldToView(0, 0);
			}
			firstUpdate = false;
		}

		double[] robotPose = Mat.decodePose(Mat.mul(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		robotToWorld = Mat.mul(Mat.translation(x, y), Mat.rotation(theta));

		if (RUN_SONAR_GUI) {
			gui.setRobotPose(x, y, theta);
		}
		logNode.getLog().info("ODOM raw: " + message.x + " " + message.y + " " + message.theta +
		                    "\nODOM processed: " +         x + " " +         y + " " +         theta);
		motorUpdate();
	}

	private void motorUpdate() {
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
			} else if (state == ALIGN_ON_BUMP) {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = transSlow;
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
		} else if (state == SPIN_ONCE_START) {
			if (!firstUpdate) {
				state = SPIN_ONCE;
			}
		} else if (state == SPIN_ONCE) {
			commandMotors.rotationalVelocity = rotFast;
			commandMotors.translationalVelocity = 0;
			if (theta > 4 * Math.PI / 3) {
				state = SPIN_ONCE_STOP;
			}
		} else if (state == SPIN_ONCE_STOP) {
			if (theta > 2 * Math.PI / 3) {
				commandMotors.rotationalVelocity = rotFast;
				commandMotors.translationalVelocity = 0;
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
			}
		}

		// publish velocity messages to move the robot
		logNode.getLog().info("MOTORs: " + commandMotors.translationalVelocity + " " + commandMotors.rotationalVelocity);
		if (state != MANUAL_MODE) {
			motorPub.publish(commandMotors);
		}
	}
	
	@Override
	public void run() {
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

		if (RUN_SONAR_GUI) {
			gui.onStart(node);
		}

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
		pointPlot = new GUIPointMsg();
		pointPlotColor = new ColorMsg();

		// initialize the ROS publication to graph points
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		linePlot = new GUILineMsg();
		linePlotColor = new ColorMsg();

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
		} else if (state == SPIN_ONCE_START) {
			stateMsg.data = "SPIN_ONCE_START";
		} else if (state == SPIN_ONCE) {
			stateMsg.data = "SPIN_ONCE";
		} else if (state == SPIN_ONCE_STOP) {
			stateMsg.data = "SPIN_ONCE_STOP";
		} else {
			stateMsg.data = "ERROR: unknown state";
		}
		statePub.publish(stateMsg);
	}
}
