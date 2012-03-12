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

	private static boolean RUN_SONAR_GUI = true;
	private SonarGUI gui;

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
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
	 * @param some shit
	 */
	public void handleSonar(/*TODO: some shit*/) {
		//TODO: some shit
	}

	
	@Override
	public void run() {
		while (true) {

			Image src = null;
			try {
			src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
			e.printStackTrace();
			}
			Pixel p = src.getPixel(width/2, height/2);

			Image dest = new Image(src.toArray(), width, height);
			//blobTrack.apply(src, dest);
			//blobTrack.classify(src, dest);

			//blobTrack.blobTracking(src, dest);
			//blobTrack.setRedThreshold(100);
			//blobTrack.setGreenThreshold(80);
			//blobTrack.setBlueThreshold(80);


			// Begin Student Code
			//Acquire sensor feedback data
			blobTrack.log_node.getLog().info("R: " + p.getRed() + "G: " + p.getGreen() + "B: " + p.getBlue());
			blobTrack.log_node.getLog().info("H: " + p.getHue() + "S: " + p.getSaturation() + "B: " + p.getBrightness());
			double sensorData[]=blobFix(blobTrack.blobPresent(src,dest));

			blobTrack.log_node.getLog().info("D: " + sensorData[0] + " X: " + sensorData[1] + " Y: " + sensorData[2]);

			if (RUN_SONAR_GUI) {
			    // update newly formed vision message
			    gui.setVisionImage(dest.toArray(), width, height);
			}



			/*Lab 4 Part 9 implemented by Daniel Gonzalez [dgonz@mit.edu]
			* -Assuming blobFix returns {range,bearing} of units {[m],[radians]}
			* --Assuming positive blob bearing means blob is to the left of the robot
			*/





			//PD control of heading		    
			headingError=headingDesired-sensorData[1];
			headingOutput=headingKp*headingError+headingKd*(headingError-headingErrorOld);

			//PD control of distance
			distanceError=sensorData[0]-distanceDesired;
			distanceOutput=distanceKp*distanceError+distanceKd*(distanceError-distanceErrorOld);

			// publish velocity messages to move the robot towards the target
			commandMotors.rotationalVelocity =headingOutput;
			commandMotors.translationalVelocity =distanceOutput;

			//update variables for the next step
			headingErrorOld=headingError;
			distanceErrorOld=distanceError;
			/*
			* To Do:
			* -Determine distanceKp, distanceKd, headingKp, headingKd via experimentation. 
			* -do ROS stuff
			*/

			k.publish(commandMotors);

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
		k= node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		commandMotors = new MotionMsg();

		// Begin Student CMotionMsg;ode

		// set parameters on blobTrack as you desire

		

		// initialize the ROS publication to command/Motors
		
		// End Student Code

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData = Image.RGB2BGR(message.data,  (int)message.width, (int)message.height);
				assert((int)message.width == width);
				assert((int)message.height == height);
				handleSonar(rgbData);
			}
		}
				);

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
