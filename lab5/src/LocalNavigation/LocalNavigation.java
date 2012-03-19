package LocalNavigation;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
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

	private static final boolean RUN_SONAR_GUI = false;
	public SonarGUI gui;

	public static final int STOP_ON_BUMP         =  0;
	public static final int ALIGN_ON_BUMP        =  1;
	public static final int ALIGNING             =  2;
	public static final int ALIGNED              =  3;
	public static final int ALIGNED_AND_ROTATING =  4;
	public static final int ALIGNED_AND_ROTATED  =  5;
	public static final int SPIN_ONCE_START      =  6;
	public static final int SPIN_ONCE            =  7;
	public static final int SPIN_ONCE_STOP       =  8;
	public static final int MANUAL_MODE          =  9;
	public static final int BACKING_UP           = 10;
	public static final int FINDING_WALL         = 11;
	public static final int TRACKING_WALL        = 12;
	public static final int WALL_ENDED           = 13;
	public static final int TURN_PREP            = 14;
	public static final int FIND_NEXT_WALL       = 15;
	public static final int DONE		     = 16;

	private int state = ALIGN_ON_BUMP;

	protected boolean firstUpdate = true;

	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	private static final double transSlow = 0.05;
	private static final double rotSlow = 0.05;
	private static final double transFast = 0.2;
	private static final double rotFast = 0.1;
	
	private ArrayList<double[]> polySegments= new ArrayList<double[]>();

	/* Frames of reference:
	 *
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 * Odometry frame: Whatever stupid frame of reference the odometry module uses, which the SonarGUI also uses
	 * Aligned frame: Relative to the robot when it most recently entered the ALIGNED state.
	 */

	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry

	private Mat wallStartRobotToWorld;
	private Mat wallEndRobotToWorld;

	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry
	private Mat worldToOdo; // initialized in handleOdometry

	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry

	// transforms between sonar and robot frames
	private static final Mat sonarToRobotRot = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.mul(Mat.translation( 0.1016, 0.2286), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.mul(Mat.translation(-0.2540, 0.2286), sonarToRobotRot);

	// transforms between aligned and world frames
	private Mat alignedToWorld; // updated when entering the ALIGNED state
	private Mat worldToAligned; // updated when entering the ALIGNED state

	private static final double wallStandoffDistance = 0.5;
	private static final double distanceToFrontOfRobot = 0.1;
	private static final double sonarRobotCenterXDifference = 0.2286;

	private LeastSquareLine lsqWorld;
	private LeastSquareLine lsqOdo;

	private boolean obstacleVisibleFront = false;
	private boolean obstacleVisibleBack  = false;

	private int obstacleVisibleFrontDebounce = 0;
	private int obstacleVisibleBackDebounce  = 0;
	private final int debounceThreshold = 20;
	
	private final boolean saveErrors = true;

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

	private Publisher<GUISegmentMsg> segmentPub;
	private GUISegmentMsg segmentPlot;
	private ColorMsg segmentPlotColor;

	private Publisher<org.ros.message.std_msgs.String> statePub;
	private org.ros.message.std_msgs.String stateMsg;

	private boolean publishTheLine = true;

	/**
	 * <p>Create a new LocalNavigation object.</p>
	 */
	public LocalNavigation() {

		setInitialParams();

		lsqWorld = new LeastSquareLine();
		lsqOdo = new LeastSquareLine();

		if (RUN_SONAR_GUI) {
			gui = new SonarGUI();
			gui.resetWorldToView(1.0,1.0,1.0);
		}
	}
	
	protected void setInitialParams() {

	}
	
	public boolean comparePoints(double	x1, double y1,double x2,double y2, double tolerance) {
		return ((Math.abs(x2-x1)<tolerance)&&(Math.abs(y2-y1)<tolerance));
	}
	
	public boolean polygonIsComplete(){
		return (comparePoints( polySegments.get(0)[0] , polySegments.get(0)[1] , 
								polySegments.get(polySegments.size()-1)[0] , polySegments.get(polySegments.size()-1)[1] ,.1));
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
			//pointPlot.color=0;
			
		} else {
			sensor = "Back";
			sonarToRobot = sonarBackToRobot;
			pointPlot.shape = 1;
			//pointPlot.color=0;
		}

		if (!firstUpdate) {
			Mat echoSonar = Mat.encodePose(message.range, 0, 0);

			Mat echoWorld = Mat.mul(robotToWorld, sonarToRobot, echoSonar);
			Mat echoOdo = Mat.mul(worldToOdo, echoWorld);

			double[] echoWorldL = Mat.decodePose(echoWorld);
			double[] echoOdoL = Mat.decodePose(echoOdo);

			if (message.range > wallStandoffDistance - sonarRobotCenterXDifference + 0.2) {
				if (message.isFront) {
					if (obstacleVisibleFrontDebounce > debounceThreshold && obstacleVisibleFront) {
						obstacleVisibleFront = false;
						obstacleVisibleFrontDebounce = 0;
					} else {
						obstacleVisibleFrontDebounce++;
					}
				} else {
					if (obstacleVisibleBackDebounce > debounceThreshold && obstacleVisibleBack) {
						obstacleVisibleBack = false;
						obstacleVisibleBackDebounce = 0;
					} else {
						obstacleVisibleBackDebounce++;
					}
				}

				pointPlotColor.r = 0;
				pointPlotColor.g = 0;
				pointPlotColor.b = 255;
			} else {
				if (message.isFront) {
					if (obstacleVisibleFrontDebounce > debounceThreshold && !obstacleVisibleFront) {
						obstacleVisibleFront = true;
						obstacleVisibleFrontDebounce = 0;
					} else {
						obstacleVisibleFrontDebounce++;
					}
				} else {
					if (obstacleVisibleBackDebounce > debounceThreshold && !obstacleVisibleBack) {
						obstacleVisibleBack = true;
						obstacleVisibleBackDebounce = 0;
					} else {
						obstacleVisibleBackDebounce++;
					}
				}

				pointPlotColor.r = 255;
				pointPlotColor.g = 0;
				pointPlotColor.b = 0;

				lsqWorld.addPoint(echoWorldL[0], echoWorldL[1]);
				lsqOdo.addPoint(echoOdoL[0], echoOdoL[1]);
				double[] line = lsqOdo.getLine();
				if (line.length > 0) {
					linePlot.lineA = line[0];
					linePlot.lineB = line[1];
					linePlot.lineC = line[2];
					linePlotColor.r = 0;
					linePlotColor.g = 130;
					linePlotColor.b = 0;
					linePlot.color = linePlotColor;
					if (publishTheLine) {
						linePub.publish(linePlot);
					}
				}
			}
			pointPlot.color = pointPlotColor;
			pointPlot.x = echoOdoL[0];
			pointPlot.y = echoOdoL[1];
			pointPub.publish(pointPlot);
		}
		//logNode.getLog().info("SONAR: Sensor: " + sensor + " Range: " + message.range);
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
		//logNode.getLog().info("BUMP: Left: " + message.left + " Right: " + message.right);
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
		//logNode.getLog().info("ODOM raw: " + message.x + " " + message.y + " " + message.theta +
		//                    "\nODOM processed: " +         x + " " +         y + " " +         theta);
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
		} else if (state == ALIGNED) {
			double[] poseAligned = Mat.decodePose(Mat.mul(worldToAligned, Mat.encodePose(x, y, theta)));
			if (poseAligned[0] > -1 * (wallStandoffDistance - distanceToFrontOfRobot)) {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = -1 * transFast;
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
				changeState(ALIGNED_AND_ROTATING);
			}
		} else if (state == ALIGNED_AND_ROTATING) {
			double[] poseAligned = Mat.decodePose(Mat.mul(worldToAligned, Mat.encodePose(x, y, theta)));
			if (poseAligned[2] > Math.PI) {
				poseAligned[2] -= 2 * Math.PI;
			}
			if (poseAligned[2] > -0.5 * Math.PI) {
				commandMotors.rotationalVelocity = -1 * rotFast;
				commandMotors.translationalVelocity = 0;
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
				changeState(ALIGNED_AND_ROTATED);
			}
		} else if (state == ALIGNED_AND_ROTATED) {
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = 0;
			changeState(BACKING_UP);
		} else if (state == BACKING_UP) {
			if (obstacleVisibleFront || obstacleVisibleBack) {
				if (lsqWorld.getNPoints() > 50 && lsqWorld.getLine().length > 0){
					commandMotors.rotationalVelocity = lineTracker(lsqWorld, false);
					commandMotors.translationalVelocity = -1 * transFast;
				} else {
					commandMotors.rotationalVelocity = 0;
					commandMotors.translationalVelocity = 0;
				}
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
				changeState(FINDING_WALL);
			}
		} else if (state == FINDING_WALL) {
			if (obstacleVisibleFront || obstacleVisibleBack) {
				changeState(TRACKING_WALL);
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = transFast;
			}
		} else if (state == TRACKING_WALL) {
			if (obstacleVisibleFront || obstacleVisibleBack) {
				commandMotors.rotationalVelocity = lineTracker(lsqWorld, true);
				commandMotors.translationalVelocity = transFast;
			} else {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
				changeState(WALL_ENDED);
			}
		} else if (state == WALL_ENDED) {
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = 0;
			if (polygonIsComplete()) {
				changeState(DONE);
			} else {
				changeState(TURN_PREP);
			}
		} else if (state==TURN_PREP){
			if(obstacleVisibleBack){
				changeState(FIND_NEXT_WALL);
			}else{
				commandMotors.rotationalVelocity =0;
				commandMotors.translationalVelocity = -1*transFast;
			}
			
		} else if (state == FIND_NEXT_WALL){
			if (bumpLeft || bumpRight) {
				commandMotors.rotationalVelocity = 0;
				commandMotors.translationalVelocity = 0;
				changeState(ALIGNING);				
			}else{
				commandMotors.rotationalVelocity = 1*rotFast;
				commandMotors.translationalVelocity = 4*transSlow;
			}
		} else if (state==DONE){
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = 0;
		}
		//------------------------------------Debug States
		else if (state == SPIN_ONCE_START) {
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
		} else if (state == DONE) {
		    commandMotors.rotationalVelocity = 0;
		    commandMotors.translationalVelocity = 0;
		}

		// publish velocity messages to move the robot
		if (state != MANUAL_MODE) {
			motorPub.publish(commandMotors);
		} else if (state == MANUAL_MODE && (bumpLeft || bumpRight)) {
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = 0;
			motorPub.publish(commandMotors);
		}
		//logNode.getLog().info("MOTORs: " + commandMotors.translationalVelocity + " " + commandMotors.rotationalVelocity);
	}

	private double lineTracker(LeastSquareLine lsq, boolean forwards) {
		double angleError;
		try {
			double distError = wallStandoffDistance - lsq.getDistance(x, y);
			double gain = 3;
			double desiredAngle = 3 * Math.PI / 2 + (forwards ? (-1 * gain * distError) : (gain * distError));
			double actualAngle = Mat.decodePose(Mat.mul(worldToAligned, Mat.encodePose(x, y, theta)))[2];
			angleError = desiredAngle - actualAngle;
			
			//saving errors for 4.3
			
			if(saveErrors) {
			    File file = new File("/home/rss-student/RSS-I-group/lab5/src/LocalNavigation/Errors.txt");
			    if (!file.exists()){
				file.createNewFile();
			    }
			    FileWriter writer = null;
			    BufferedWriter out = null;
				writer = new FileWriter(file,true);
				out = new BufferedWriter(writer);
				String toBeWritten = System.currentTimeMillis() + " " +
				    distError + " " + angleError;
				out.append(toBeWritten);
				out.newLine();

				    out.close();
				    writer.close();



			}
			
			
			if (angleError > Math.PI / 6) {
				angleError = Math.PI / 6;
			} else if (angleError < -1 * Math.PI / 6) {
				angleError = -1 * Math.PI / 6;
			}
			if (angleError > Math.PI) {
				angleError -= 2 * Math.PI;
			}
			//			logNode.getLog().info("STEERING: " + distError +" "+ desiredAngle +" "+ actualAngle +" "+ angleError);
		} catch (Exception e) {
		    logNode.getLog().info("getDistance threw an exception.\n"+ e.toString());
			angleError = 0;
		}
		return 0.4 * angleError;
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

		// initialize the ROS publication to graph lines
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		linePlot = new GUILineMsg();
		linePlotColor = new ColorMsg();

		// initialize the ROS publication to graph line segments
		segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");
		segmentPlot = new GUISegmentMsg();
		segmentPlotColor = new ColorMsg();

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
			worldToAligned = Mat.mul(Mat.rotation(-theta), Mat.translation(-x, -y));
			alignedToWorld = Mat.inverse(worldToAligned);
			stateMsg.data = "ALIGNED";
		} else if (state == ALIGNED_AND_ROTATING) {
			stateMsg.data = "ALIGNED_AND_ROTATING";
		} else if (state == ALIGNED_AND_ROTATED) {
			stateMsg.data = "ALIGNED_AND_ROTATED";
		} else if (state == SPIN_ONCE_START) {			stateMsg.data = "SPIN_ONCE_START";
		} else if (state == SPIN_ONCE) {
			stateMsg.data = "SPIN_ONCE";
		} else if (state == SPIN_ONCE_STOP) {
			stateMsg.data = "SPIN_ONCE_STOP";
		} else if (state == BACKING_UP) {
			lsqWorld.reset();
			lsqOdo.reset();
			stateMsg.data = "BACKING_UP";
		} else if (state == FINDING_WALL) {
			stateMsg.data = "FINDING_WALL";
		} else if (state == TRACKING_WALL) {
			wallStartRobotToWorld = robotToWorld;
			lsqWorld.reset();
			lsqOdo.reset();
			stateMsg.data = "TRACKING_WALL";
		} else if (state == WALL_ENDED) {
			wallEndRobotToWorld = robotToWorld;
			double line[] = lsqOdo.getLine();

			double[] sonarOdoStartxyL   = Mat.decodePose(Mat.mul(worldToOdo, wallStartRobotToWorld, sonarFrontToRobot, Mat.encodePose(0, 0, 0)));
			double[] sonarOdoStartdxdyL = Mat.decodePose(Mat.mul(worldToOdo, wallStartRobotToWorld, sonarFrontToRobot, Mat.encodePose(0.5, 0, 0)));

			// refers to sonar vectors
			double xStart = sonarOdoStartxyL[0];
			double yStart = sonarOdoStartxyL[1];
			double dxStart = sonarOdoStartdxdyL[0];
			double dyStart = sonarOdoStartdxdyL[1];
			dxStart -= xStart;
			dyStart -= yStart;

			// refers to wall location
			double xyStart[] = lineVectorIntersection(line, xStart, yStart, dxStart, dyStart);

			double[] sonarOdoEndxyL   = Mat.decodePose(Mat.mul(worldToOdo, wallEndRobotToWorld, sonarBackToRobot, Mat.encodePose(0, 0, 0)));
			double[] sonarOdoEnddxdyL = Mat.decodePose(Mat.mul(worldToOdo, wallEndRobotToWorld, sonarBackToRobot, Mat.encodePose(0.5, 0, 0)));

			// refers to sonar vectors
			double xEnd = sonarOdoEndxyL[0];
			double yEnd = sonarOdoEndxyL[1];
			double dxEnd = sonarOdoEnddxdyL[0];
			double dyEnd = sonarOdoEnddxdyL[1];
			dxEnd -= xEnd;
			dyEnd -= yEnd;

			// refers to wall location
			double xyEnd[] = lineVectorIntersection(line, xEnd, yEnd, dxEnd, dyEnd);
			
			polySegments.add(new double[] {xStart,yStart,xEnd,yEnd});
			
			segmentPlot.startX = xyStart[0];
			segmentPlot.startY = xyStart[1];
			segmentPlot.endX = xyEnd[0];
			segmentPlot.endY = xyEnd[1];
			
			//random color from SonarGUI
			Color randomColor = gui.makeRandomColor();
			segmentPlotColor.r = randomColor.getRed();
			segmentPlotColor.g = randomColor.getGreen();
			segmentPlotColor.b = randomColor.getBlue();
			
			segmentPlot.color = segmentPlotColor; 

			publishTheLine = false;
			segmentPub.publish(segmentPlot);
			
			stateMsg.data = "WALL_ENDED";
		} else if (state == TURN_PREP) {
			stateMsg.data = "TURN_PREP";
		} else if (state == FIND_NEXT_WALL) {
			stateMsg.data = "FIND_NEXT_WALL";
		} else if (state == DONE) {
			stateMsg.data = "DONE";
		} else {
			stateMsg.data = "ERROR: unknown state";
		}
		statePub.publish(stateMsg);
	}

	public static double[] lineVectorIntersection(double[] line, double x, double y, double dx, double dy) {
		double a = line[0];
		double b = line[1];
		double c = line[2];
		double p = (-a * x - b * y - c) / (a * dx + b * dy);
		return new double[] {x + p * dx, y + p * dy};
	}
}
