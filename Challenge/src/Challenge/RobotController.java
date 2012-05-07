package Challenge;

// imports that probably aren't needed are commented out

/*
import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
*/
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
/*
import java.util.Scanner;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
*/

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
/*
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.ColorMsg;
*/
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import LocalNavigation.Mat;
/*
import VisualServo.VisionGUI;
*/

public class RobotController implements NodeMain, Runnable  {
	private Node logNode;
	
	//State stuff
	protected boolean firstUpdate = true;
	public static final int IDLE=0;
	public static final int ROTATING=1;
	public static final int TRANSLATING=2;
	private int state = IDLE;
	
	//PI Parameters
	private double distanceError=0;
	private double distanceErrorIntegral=0;
	private double distanceKp=.15;
	private double distanceKi=0.002;
	private double distanceOutput=0;
	private double headingDesired=0;
	private double headingError=0;
	private double headingErrorIntegral=0;
	private double headingKp=.15;
	private double headingKi=0.005;
	private double headingOutput=0;
	
	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry
	
	//Waypoint list
	ArrayList<Waypoint> myWaypoints=new ArrayList<Waypoint>();

	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry
	private Mat worldToOdo; // initialized in handleOdometry
	
	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry
	
	//ROS stuff to read odometry, command the motors and publish our current state
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	
	public Publisher<MotionMsg> motorPub;
	private MotionMsg commandMotors;

	private Publisher<org.ros.message.std_msgs.String> statePub;
	private org.ros.message.std_msgs.String stateMsg;

	private TwoMice mice = new TwoMice();
	
	/**
	 * <p>
	 * Add a waypoint to our queue of waypoints.
	 * </p>
	 *
	 * @param p The Waypoint.
	 */
	public void addWaypoint(Waypoint p){
		myWaypoints.add(p);
	}
	
	public double fixAngle(double theta){
		while (theta>=2*Math.PI){
			theta-=2*Math.PI;
		}
		while (theta<0){
			theta+=2*Math.PI;
		}
		return theta;
	}
	public double fixAngle2(double theta){
		while (theta>=Math.PI){
			theta-=2*Math.PI;
		}
		while (theta<-Math.PI){
			theta+=2*Math.PI;
		}
		return theta;
	}
	/**
	 * <p>Handle an OdometryMsg.</p>
	 *
	 * @param the message
	 */
	public void handleOdometry(org.ros.message.rss_msgs.OdometryMsg message) {
		System.err.println("handling odometry");
		if ( firstUpdate ) {
			odoToWorld = Mat.mul(Mat.rotation(-message.theta), Mat.translation(-message.x, -message.y));
			worldToOdo = Mat.inverse(odoToWorld);
			
			firstUpdate = false;
		}

		double[] robotPose = Mat.decodePose(Mat.mul(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		System.err.println("(" + x + " " + y + " " + theta + ")");

		theta=fixAngle(theta);

		robotToWorld = Mat.mul(Mat.translation(x, y), Mat.rotation(theta));

		//logNode.getLog().info("ODOM raw: " + message.x + " " + message.y + " " + message.theta +
		//                    "\nODOM processed: " +         x + " " +         y + " " +         theta);
		motorUpdate();
	}
	
	public double getX(){
		return x;
	}
	public double getY(){
		return y;
	}
	public void setTranslationalVelocity(double v){
		commandMotors.rotationalVelocity=v;
		motorPub.publish(commandMotors);
	}
	
	
	//Set of useful functions
	public boolean comparePoints(double x1, double y1, double x2, double y2, double tolerance) {
		return Math.pow(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2), 0.5) < tolerance;
	}
	
	public double getAngle(double x1, double y1, double x2, double y2) {
		return fixAngle(Math.atan2(y2-y1, x2-x1));
	}
	
	public boolean compareAngles(double thetaOne, double thetaTwo, double tolerance) {
		return Math.abs(thetaTwo-thetaOne) < tolerance;
	}
	
	public double getDistance(double x1, double y1, double x2, double y2){
		return Math.pow(Math.pow(x2-x1, 2)+Math.pow(y2-y1, 2), .5);
	}
	
	
	// Main FSM handler
	private void motorUpdate(){
		if (state==IDLE){
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = 0;
			if(myWaypoints.size()>0){
				
				//Set the desired heading, taking into account the direction we wish to face
				if(myWaypoints.get(0).getDir()==1){
					headingDesired=fixAngle(getAngle(x,y,myWaypoints.get(0).getX(),myWaypoints.get(0).getY()));
				}else{
					headingDesired=fixAngle(getAngle(myWaypoints.get(0).getX(),myWaypoints.get(0).getY(),x,y));
				}
				//logNode.getLog().info("Turning Towards "+myWaypoints.get(0).getX()+" , "+myWaypoints.get(0).getY());
				changeState(ROTATING);
			}
		}else if(state==ROTATING){
			headingError = fixAngle2(headingDesired - theta);
			
			commandMotors.translationalVelocity = 0;
			commandMotors.rotationalVelocity = headingKp * headingError + headingKi * headingErrorIntegral;
			headingErrorIntegral+=headingError;
			
			if(compareAngles(theta,headingDesired, .15)){
				commandMotors.rotationalVelocity = 0;
				headingErrorIntegral=0;
				//logNode.getLog().info("Moving to "+myWaypoints.get(0).getX()+" , "+myWaypoints.get(0).getY());
				changeState(TRANSLATING);
			}

		}else if(state==TRANSLATING){
			distanceError=getDistance(x, y, myWaypoints.get(0).getX() ,myWaypoints.get(0).getY() );
			
			commandMotors.rotationalVelocity = 0;
			commandMotors.translationalVelocity = myWaypoints.get(0).getDir()*(distanceKp * distanceError + distanceKi * distanceErrorIntegral);
			distanceErrorIntegral += distanceError;
			
			if(comparePoints(x,y,myWaypoints.get(0).getX(),myWaypoints.get(0).getY(), .2 )){
				distanceErrorIntegral=0;
				commandMotors.translationalVelocity = 0;
				myWaypoints.remove(0);
				changeState(IDLE);
			}
		}
	motorPub.publish(commandMotors);
	}
	
	@Override
	public void run() {
	}
	
	/**
	 * <p>
	 * Run the RobotController process
	 * </p>
	 *
	 * @param optional
	 *   command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		logNode = node;
		logNode.getLog().info("RobotController Online");
		// initialize the ROS subscription to rss/mouseOdometry
		odoSub = node.newSubscriber("/rss/mouseOdometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
					handleOdometry(message);
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
		
/*		addWaypoint(new Waypoint(.5, 0, (short) 1));
		addWaypoint(new Waypoint(1, 1, (short) 1));
		addWaypoint(new Waypoint(0, 0, (short) -1));
		addWaypoint(new Waypoint(1, 0, (short) 1));
		addWaypoint(new Waypoint(1, 1, (short) 1));
		addWaypoint(new Waypoint(0, 1, (short) 1));
		addWaypoint(new Waypoint(0, 0, (short) 1));
		addWaypoint(new Waypoint(0, 1, (short) -1));
		addWaypoint(new Waypoint(1, 1, (short) -1));*/
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

	public GraphName getDefaultNodeName() {
		return new GraphName("rss/RobotController");
	}

	private void changeState(int newState){
		state=newState;
		if(state==IDLE){
			stateMsg.data = "IDLE";
		}else if (state==ROTATING){
			stateMsg.data = "ROTATING";
			System.err.println("Arrived at waypoint.");
		}else if (state==TRANSLATING){
			stateMsg.data = "TRANSLATING";
		}
		

		statePub.publish(stateMsg);
	}
	
	
	
	}
