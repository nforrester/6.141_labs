package Challenge.PhaseTwo;

import Challenge.Mat;
import Challenge.MessageListener;
import Challenge.MotionMsg;
import Challenge.Node;
import Challenge.Publisher;
import Challenge.Subscriber;

public class PhaseTwoMotorController implements nodeMain, Runnable {
	
	private Node logNode;
	
	//ROS stuff to read odometry, command the motors and publish our current state
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	
	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry
	
	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry
	private Mat worldToOdo; // initialized in handleOdometry
	
	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry
	
	//ROS stuff to read odometry, command the motors and publish our current state
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	
	public Publisher<MotionMsg> motorPub;
	private MotionMsg commandMotors;
	
	private boolean isMoving=false;
	
	//PI parameters
	private double distanceError=0;
	private double distanceErrorIntegral=0;
	private double distanceKp=.65;
	private double distanceKi=0.005;
	
	
	//Set of useful functions
	public boolean comparePoints(double x1, double y1, double x2, double y2, double tolerance) {
		return Math.pow(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2), 0.5) < tolerance;
	}
	public double getDistance(double x1, double y1, double x2, double y2){
		return Math.pow(Math.pow(x2-x1, 2)+Math.pow(y2-y1, 2), .5);
	}
	
	//Getter Methods
	public boolean getIsMoving(){
		return isMoving;
	}
	public double getX(){
		return x;
	}
	
	//Main Command Functions
	public void goToX(double xNew){
		isMoving=true;
		System.err.println("Translating");
		while(!comparePoints(x,y,xNew,y,.02)){
		distanceError=getDistance(x, y, xNew ,y);
		
		commandMotors.translationalVelocity = 1*(distanceKp * distanceError + distanceKi * distanceErrorIntegral);
		distanceErrorIntegral += distanceError;
		
		motorPub.publish(commandMotors);
		}
		distanceErrorIntegral=0;
		commandMotors.translationalVelocity = 0;
		motorPub.publish(commandMotors);
		isMoving=false;
	}
	
	public void goToRelX(double dx){
		goToX(x+dx);
	}
	
	public void setTranslationalVelocity(double v){
		if(v==0){
			isMoving=false;
		}
		else{
			isMoving=true;
		}
		commandMotors.translationalVelocity=v;
		motorPub.publish(commandMotors);
	}
	
	
	//loop stuff	
	public void handleOdometry(org.ros.message.rss_msgs.OdometryMsg message) {
		if ( firstUpdate ) {
			odoToWorld = Mat.mul(Mat.rotation(-message.theta), Mat.translation(-message.x, -message.y));
			worldToOdo = Mat.inverse(odoToWorld);
			
			firstUpdate = false;
		}

		double[] robotPose = Mat.decodePose(Mat.mul(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		//System.err.println("(odometry " + x + " " + y + " " + theta + ")");

		theta=fixAngle(theta);

		robotToWorld = Mat.mul(Mat.translation(x, y), Mat.rotation(theta));

		//logNode.getLog().info("ODOM raw: " + message.x + " " + message.y + " " + message.theta +
		//                    "\nODOM processed: " +         x + " " +         y + " " +         theta);
		//motorUpdate();
	}
	public void onStart(Node node){
		
		//Initialize the ros subscription to odometry
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
		Thread runningStuff = new Thread(this);
		runningStuff.start();
		
		
	}
	@Override
	public void run() {
				
	}

}
