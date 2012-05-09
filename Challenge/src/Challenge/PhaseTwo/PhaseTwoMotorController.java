package Challenge.PhaseTwo;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import LocalNavigation.Mat;

public class PhaseTwoMotorController implements NodeMain, Runnable {
	
	private Node logNode;
	
	//ROS stuff to read odometry, command the motors and publish our current state
	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	
	// x, y, and theta record the robot's current position in the world frame
	private double x;     // continuously updated in handleOdometry
	private double y;     // continuously updated in handleOdometry
	private double theta; // continuously updated in handleOdometry
	

	protected boolean firstUpdate = true;
	
	// transforms between odometry and world frames
	private Mat odoToWorld; // initialized in handleOdometry
	private Mat worldToOdo; // initialized in handleOdometry
	
	// transforms between robot and world frames
	private Mat robotToWorld; // continuously updated in handleOdometry
	
	
	public Publisher<MotionMsg> motorPub;
	private MotionMsg commandMotors;
	
	private boolean isMoving=false;
	
	//PI parameters
	private double distanceError=0;
	private double distanceErrorIntegral=0;
	private double distanceKp=.3;
	private double distanceKi=0.01;
	
	private boolean newOdoMsg=false;
	
	
	//Set of useful functions
	public boolean comparePoints(double x1, double y1, double x2, double y2, double tolerance) {
		return Math.pow(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2), 0.5) < tolerance;
	}
	public double getDistance(double x1, double y1, double x2, double y2){
		return Math.pow(Math.pow(x2-x1, 2)+Math.pow(y2-y1, 2), .5);
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
	
	//Getter Methods
	public boolean getIsMoving(){
		return isMoving;
	}
	public double getX(){
		return x;
	}
	
	//Main Command Functions
	public void goToX(double xNew, int d){
		isMoving=true;
		System.err.println("Translating to"+xNew);
		
		while(!comparePoints(x,y,xNew,y,.01)){
			if(newOdoMsg == true){
				newOdoMsg = false;
				distanceError=d*getDistance(x, y, xNew ,y);
				
				commandMotors.translationalVelocity = (distanceKp * distanceError + distanceKi * distanceErrorIntegral);
				distanceErrorIntegral += distanceError;
				
				System.err.println("X ("+x+") - xNew ("+xNew+") = distanceError = " + distanceError);
				System.err.println("velocity = " + commandMotors.translationalVelocity);
				motorPub.publish(commandMotors);
			}
			Thread.yield();
		}
		distanceErrorIntegral=0;
		commandMotors.translationalVelocity = 0;
		motorPub.publish(commandMotors);
		isMoving=false;
		
	}
	
	public void goToRelX(double dx, int dir){
		goToX(x+dx,dir);
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
		newOdoMsg=true;
		double[] robotPose = Mat.decodePose(Mat.mul(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		System.err.println("(odometry " + x + " " + y + " " + theta + ")");

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
	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
