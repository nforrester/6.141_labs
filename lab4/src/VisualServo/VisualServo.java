package VisualServo;

import java.util.concurrent.ArrayBlockingQueue;


import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import VisualServo.Image.Pixel;

import MotorControlSolution.*;

/**
 * 
 * @author previous TA's, prentice, vona
 *
 */
public class VisualServo implements NodeMain, Runnable{

	private static final int width = 160;

	private static final int height = 120;
	
    //Lab 4 part 9 parameters
    private double distanceDesired=.5;
    private double distanceError=0;
    private double distanceErrorOld=0;
    private double distanceKp=5;
    private double distanceKd=1;
    private double distanceOutput=0;
    private double headingDesired=0;
    private double headingError=0;
    private double headingErrorOld=0;
    private double headingKp=5;
    private double headingKd=1;
    private double headingOutput=0;
    private double sensorData[]={0,0};
    protected RobotVelocityController robotVelocityController;
    protected RobotBase robot = new RobotBase();

	/**
	 * <p>The blob tracker.</p>
	 **/
	private BlobTracking blobTrack = null;



	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	/**
	 * <p>Create a new VisualServo object.</p>
	 */
	public VisualServo() {

		setInitialParams();

		gui = new VisionGUI();
	}
	
	/**
	 * <p>Converts the area and position of the ball in the image</p>
	 *
	 * @param input The desired blobPresent output to convert, assuming blobPresent returns (area,x,y) of units ([px^2],[px],[px]).
	 */
	public double[] blobFix(int [] input){		
		
		/*
		 * Measurements of environment:
		 * Physical ball diameter = 4.000 [inches] or 101.60 [mm]
		 * Pixel Area of ball 1 meter away = n [px] 
		 */
		double output[]=new double[2];
		//Scaling factors for area,x, and y. Found through experimentation.
		double kA=1; //units [m]/[px^2]
		double kX=1; //units [m]/[px] normalized to exactly 1 meter away
		double kY=1; //units [m]/[px] normalized to exactly 1 meter away
		
		double area=(double)input[0];
		double x=(double)input[1];
		double y=(double)input[2];
		
		/*
		 * toDo: 
		 * -make more robust calculations based on camera lens angle specs.
		 * -Edit kA, kX, kY after experimentation.
		 */
		
		if (area==0){
			output[0]=.5;
			output[1]=kX*x;
		}
		else{
			output[0]=kA*area;
			output[1]=kX*x;
		}
		return output;
	}

	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a CameraMessage. Perform blob tracking and
	 * servo robot towards target.</p>
	 * 
	 * @param a received camera message 
	 */
	public void handle(byte[] rawImage) {

		visionImage.offer(rawImage);		
	}
	
	  /**
	   * <p>Convenience method to {@link
	   * RobotVelocityController#setDesiredAngularVelocity} on {@link
	   * #robotVelocityController}.</p>
	   *
	   * @param left the desired left wheel angular velocity in rad/s, positive
	   * means corresp side moves forward
	   * @param right the desired right wheel angular velocity in rad/s, positive
	   * means corresp side moves forward
	   **/
	  protected void setDesiredAngularVelocity(double left, double right) {
	    robotVelocityController.setDesiredAngularVelocity(left, right);
	  }	
	
	@Override
	public void run(){
		while(true){

		    Image src = null;
		    try {
			src = new Image(visionImage.take(), width, height);
		    } catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		    }
		    Pixel p = src.getPixel(width/2, height/2);
		    
		    Image dest = new Image(src.toArray(), width, height);
		    //blobTrack.apply(src, dest);
		    //blobTrack.classify(src, dest);
		    
		    //blobTrack.blobTracking(src, dest);
		    
		    
		    // update newly formed vision message
		    gui.setVisionImage(dest.toArray(), width, height);
		    
		    // Begin Student Code
		    
		    /*Lab 4 Part 9 implemented by Daniel Gonzalez [dgonz@mit.edu]
		     * -Assuming blobFix returns {range,bearing} of units {[m],[radians]}
		     * --Assuming positive blob bearing means blob is to the left of the robot
		     */
		    
		    //Acquire sensor feedback data
		    double sensorData[]=blobFix(blobTrack.blobPresent(src,dest));
		    
		    //PD control of heading		    
		    headingError=headingDesired-sensorData[1];
		    headingOutput=headingKp*headingError+headingKd*(headingError-headingErrorOld);
		    
		    //PD control of distance
		    distanceError=distanceDesired-sensorData[0];
		    distanceOutput=distanceKp*distanceError+distanceKd*(distanceError-distanceErrorOld);
		    
		    // publish velocity messages to move the robot towards the target
		    setDesiredAngularVelocity(distanceOutput+headingOutput, distanceOutput-headingOutput);
		    
		    //update variables for the next step
		    headingErrorOld=headingError;
		    distanceErrorOld=distanceError;
		    /*
		     * To Do:
		     * -Determine distanceKp, distanceKd, headingKp, headingKd via experimentation. 
		     * -do ROS stuff
		     */
		    
		    // End Student Code
		}
	}

	/**
	 * <p>
	 * Run the VisualServo process
	 * </p>
	 * 
	 * @param optional
	 *            command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		blobTrack = new BlobTracking(width, height);
		blobTrack.log_node = node;
		// Begin Student Code

		// set parameters on blobTrack as you desire

		

		// initialize the ROS publication to command/Motors
	    this.robotVelocityController = new RobotVelocityController(new WheelVelocityControllerI(), new WheelVelocityControllerI());
	    this.robot.setRobotVelocityController(robotVelocityController);
	    this.robot.enableMotors(true);

		// End Student Code

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData = Image.RGB2BGR(message.data,  (int)message.width, (int)message.height);
				assert((int)message.width == width);
				assert((int)message.height == height);
				handle(rgbData);
			}
		}
				);

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if ( firstUpdate ) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
			}
		}
				);
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
		return new GraphName("rss/visualservo");
	}
}
