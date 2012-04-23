/*
 * 
 */
package Grasping;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


import VisualServo.Image;
import VisualServo.VisionGUI;
import LocalNavigation.Mat;
import VisualServo.BlobTracking;

import java.util.LinkedList;

public class Grasping implements NodeMain{
	public static double[] targetAngles = {0.0,0.0,0.0};

	long startTime = 0;

	private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;
	private Subscriber<org.ros.message.sensor_msgs.Image> rssVideoSub;
	private Publisher<org.ros.message.sensor_msgs.Image> vidPub;

	private BlobTracking blobTrack = null;
	private VisionGUI gui;

	private static boolean setyes = false;

	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	private static final int width = 160;
	private static final int height = 120;

	//////////////////////////////////////////////////////////////////////////////////////////////
	/* Block pick up macro variables */
	private static boolean armInPickingPosition = false;
	private static boolean armInPickingPlace = false;
	private static boolean blockPickedupAndMoveForward = false;
	private static boolean placeBlockdown = false;
	////////////////////////////////////////////////////////////////////////////////////////////

	JointController jc;

	private static LinkedList<double[]> taskList = new LinkedList<double[]>();

	private static void prepareToPickUpBlock() {
		taskList.add(new double[] { 0, 1.3859967589366735, 0.599758597503506});
		taskList.add(new double[] { -1.9332877868244882, 1.3859967589366735, 0.599758597503506});
	}

	private static void pickUpBlock() {
		taskList.add(new double[] { -1.9332877868244882, 1.3859967589366735, 0});
		taskList.add(new double[] {0, 1.3859967589366735, 0});
		taskList.add(new double[] {0, 0, 0});
	}

	private static void putDownBlock() {
		taskList.add(new double[] { 0, 1.3859967589366735, 0});
		taskList.add(new double[] { -1.9332877868244882, 1.3859967589366735, 0});
		taskList.add(new double[] { -1.9332877868244882, 1.3859967589366735, 0.599758597503506});
	}

    private static void openGripper() {
	taskList.add( new double[] {taskList.getLast()[0], taskList.getLast()[1], 0.599758597503506});
    }

    private static void closeGripper() {
	taskList.add( new double[] {taskList.getLast()[0], taskList.getLast()[1], 0});
    }

    private static void moveUp(double angle) {
	taskList.add( new double[] {angle, taskList.getLast()[1], taskList.getLast()[2]});
    }
    
    private static void bendElbow(double angle) {
	taskList.add( new double[] {taskList.getLast()[0], angle, taskList.getLast()[2]});
    }

    private static void moveToGround() {
	taskList.add(new double[] { -1.9332877868244882, 1.3859967589366735, 0.599758597503506});
    }

	public static void setServoAngles(double[] desiredAngles){
		targetAngles = new double[] {desiredAngles[0],desiredAngles[1],desiredAngles[2]};
	}

	public static double[] inverseKinematics(double x2, double z2) {
		// Please excuse the variable names. For an explanation, see:
		// https://courses.csail.mit.edu/rss/wiki/index.php?title=Team6Lab72012#Arm_control_and_inverse_kinematics

		double x0 = .07; // rough estimate, TODO: replace with measurements!!!!
		double z0 = .12; // rough estimate, TODO: replace with measurements!!!!
		Mat p0 = encodePoint2(x0, z0);
		Mat p2 = encodePoint2(x2, z2);

		double l1 = .25; // rough estimate, TODO: replace with measurements!!!!
		double l2 = .15; // rough estimate, TODO: replace with measurements!!!!

		double phi = Math.atan2(z2 - z0, x2 - x0);
		double d = Mat.l2(Mat.add(p2, Mat.mul(-1, p0)));

		double x1_ = (d*d - l2*l2 + l1*l1) / (2 * d);
		double z1_ = Math.pow(4 * d*d * l1*l1 - Math.pow(d*d - l2*l2 + l1*l1, 2), 0.5) / (2 * d);

		double phi_ = Math.atan2(z1_, x1_);

		double theta1 = phi + phi_;
		double theta2 = Math.acos((x2 - x0 - l1 * Math.cos(theta1)) / l2) - theta1;

		return new double[]{theta1, theta2};
	}

	public static Mat encodePoint2(double x, double y) {
		Mat p = new Mat(2, 1);
		p.data[0][0] = x;
		p.data[1][0] = y;
		return p;
	}

	public static double[] decodePoint2(Mat p) {
		double x = p.data[0][0];
		double y = p.data[1][0];
		return new double[]{x, y};
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
	public void onStart(Node node) {
		jc = new JointController(node);

		gui = new VisionGUI();

		blobTrack = new BlobTracking(width, height);

		armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
		armSub.addMessageListener(new ArmListener());

		rssVideoSub = node.newSubscriber("rss/video", "sensor_msgs/Image");
		rssVideoSub.addMessageListener(new videoListener());

		vidPub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image");

		prepareToPickUpBlock();
		pickUpBlock();
		putDownBlock();
	}

	@Override
	public GraphName getDefaultNodeName() {
		return null;
	}

	public class ArmListener implements MessageListener<ArmMsg> {

		@Override public void onNewMessage(ArmMsg currentMessage) {

			//////////////////////////////////////////////////////////////////////////////////////////////
			/* Block pick up and move macro  */
			 if(!armInPickingPosition){
				 Grasping.prepareToPickUpBlock();
				 armInPickingPosition = true;
			 }

			System.err.println("Arm listener running!");

			if (!taskList.isEmpty()) {
				System.err.println("We've got work to do!");
				double currentShoulderAngle = ShoulderController.getAngleEquivalent(currentMessage.pwms[0]);
				double currentWristAngle = WristController.getAngleEquivalent(currentMessage.pwms[1]);
				double currentGripperAngle = GripperController.getAngleEquivalent(currentMessage.pwms[2]);

				if ((currentShoulderAngle != targetAngles[0]) ||
				    (currentWristAngle    != targetAngles[1]) ||
				    (currentGripperAngle  != targetAngles[2])) {
					jc.commandServos(currentMessage, targetAngles);
					System.err.println("Target angles: " + targetAngles[0] + " " + targetAngles[1] + " " + targetAngles[2]);
				}

				setServoAngles(taskList.get(0));
				if (compareCurrentPositionWithAngles(currentMessage, taskList.get(0))) {
					taskList.remove(0);
					setServoAngles(taskList.get(0));
					System.err.println("Set target angles: " + targetAngles[0] + " " + targetAngles[1] + " " + targetAngles[2]);
				}
			}
		}
	}

	public class videoListener implements MessageListener<org.ros.message.sensor_msgs.Image> {

		@Override public void onNewMessage(org.ros.message.sensor_msgs.Image newImage) {


			byte[] rgbData = Image.RGB2BGR(newImage.data, (int)newImage.width, (int)newImage.height);

			synchronized (visionImage){
				visionImage.offer(rgbData);
			}


			Image src = null;

			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}


			Image dest = new Image(src);
			int[] currentPositions = blobTrack.blobPresent(src,dest);

			System.err.println("Area -> " + currentPositions[0] + " X -> " + currentPositions[1] + " Y -> " + currentPositions[2]);

			org.ros.message.sensor_msgs.Image pubImage =	new org.ros.message.sensor_msgs.Image();
			pubImage.width = width;
			pubImage.height = height;
			pubImage.encoding = "rgb8";
			pubImage.is_bigendian = 0;
			pubImage.step = width*3;
			pubImage.data = dest.toArray();
			vidPub.publish(pubImage);
		}
	}




	private static boolean compareCurrentPositionWithAngles(ArmMsg currentMessage,double[] compareAngles){
		System.err.print("Shoulder : " + ShoulderController.getAngleEquivalent(currentMessage.pwms[0]) + " ");
		System.err.print("Wrist : " + WristController.getAngleEquivalent(currentMessage.pwms[1]) + " ");
		System.err.println("Gripper : " + GripperController.getAngleEquivalent(currentMessage.pwms[2]));
		return Math.abs(ShoulderController.getAngleEquivalent(currentMessage.pwms[0])-compareAngles[0]) < 0.05 &&
		       Math.abs(WristController.getAngleEquivalent(currentMessage.pwms[1])-compareAngles[1])    < 0.05 &&
		       Math.abs(GripperController.getAngleEquivalent(currentMessage.pwms[2])-compareAngles[2])  < 0.05;
	}
}
