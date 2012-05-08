package Challenge;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Controller.Type;

import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;

import java.io.PrintWriter;
import java.io.FileWriter;
import java.io.IOException;

public class TwoMice implements NodeMain {
	//put this in VM arguments before running
	// -Djava.library.path=dist/

	private Node thisNode;
	public static final String APPNAME = "TwoMice";

	private Controller mouseOne = null;
	private Controller mouseTwo = null;

	private Controller keyboard = null;

	private final String mouseName = "DELL DELL USB Laser Mouse";

	private double mouseOneX = 0;
	private double mouseOneY = 0;

	private double mouseTwoX = 0;
	private double mouseTwoY = 0;

	private double mouseOneDeltaX = 0;
	private double mouseOneDeltaY = 0;

	private double mouseTwoDeltaX = 0;
	private double mouseTwoDeltaY = 0;

	private double x;
	private double y;
	private double theta;
	private double TICKS_PER_METER = 20958; // needs to be measured, not guessed
	private double DISTANCE_BETWEEN_SENSORS = .200; // meters
	private double SENSOR_Y_OFFSET = .035; // meters

	private Thread mouseMonitorThread;
	private Thread odometryPublisherThread;

	private Publisher<OdometryMsg> pub;
	private OdometryMsg msg = new OdometryMsg();

	public TwoMice() {
	}

	public void onStart(Node node) {
		System.err.println("TWOMICE INITIALIZING");
		thisNode = node;

		x     = -1 * SENSOR_Y_OFFSET;
		y     = 0;
		theta = 0;

		Controller[] ca = ControllerEnvironment.getDefaultEnvironment().getControllers();

		for (Controller controller : ca) {
			if (controller.getType() == Type.MOUSE) {
				if(mouseOne == null && controller.getName().trim().equals(mouseName)) {
					mouseOne = controller;
					System.out.println(mouseOne.getName());
				}
				else if(controller.getName().trim().equals(mouseName)) {
					mouseTwo = controller;
				}
			} else if (controller.getType() == Type.KEYBOARD) {
				keyboard = controller;
			}
		}

		// throw exception if mouse not present
		if (mouseOne == null ) {
			throw new NullPointerException("You don't have a mouse");
		}
		else if(keyboard == null) {
			throw new NullPointerException("You don't have a keyboard");
		}

		System.err.println("LAUNCHING TWOMICE MONITOR");

		mouseMonitorThread = new Thread(new Runnable() {
				@Override
				public void run() {
					System.err.println("Mouse monitor thread starting");
					while (true) {
						mouseOne.poll();
						Component[] componentsOne = mouseOne.getComponents();

						for(int i=0;i< componentsOne.length;i++) {
							if(componentsOne[i].isAnalog()) {
								if(componentsOne[i].getName().equals("x")) {
									mouseOneDeltaX = (double)componentsOne[i].getPollData();
									mouseOneX += mouseOneDeltaX;
								}
								else if(componentsOne[i].getName().equals("y")) {
									mouseOneDeltaY = (double)componentsOne[i].getPollData();
									mouseOneY += mouseOneDeltaY;
								}
							}
						}

						mouseTwo.poll();
						Component[] componentsTwo = mouseTwo.getComponents();

						for(int i=0;i< componentsTwo.length;i++) {
							if(componentsTwo[i].isAnalog()) {
								if(componentsTwo[i].getName().equals("x")) {
									mouseTwoDeltaX = (double)componentsTwo[i].getPollData();
									mouseTwoX += mouseTwoDeltaX;
								}
								else if(componentsTwo[i].getName().equals("y")) {
									mouseTwoDeltaY = (double)componentsTwo[i].getPollData();
									mouseTwoY += mouseTwoDeltaY;
								}
							}
						}

						// call the computeOdometry method here
						// mouseOne is left and MouseTwo is right
						computeOdometry(getMouseOneDeltaX(),getMouseOneDeltaY(),getMouseTwoDeltaX(),getMouseTwoDeltaY());

						try {
							Thread.sleep(20);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				}
			});
		mouseMonitorThread.start();
		System.err.println("LAUNCHING ODOMETRY PUBLISHER");
		odometryPublisherThread = new Thread(new Runnable() {
				@Override
				public void run() {
					System.err.println("publisher thread starting");
					pub = thisNode.newPublisher("/rss/mouseOdometry", "rss_msgs/OdometryMsg");

					while (true) {
						publishOdometry();

						try {
							Thread.sleep(100);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				}
			});
		odometryPublisherThread.start();
		System.err.println("ALL TWOMICE THREAD LAUNCHED");
	}


	private double getMouseOneX() {
		return mouseOneX;
	}

	private double getMouseOneY() {
		return -mouseOneY;
	}

	private double getMouseTwoX() {
		return mouseTwoX;
	}

	private double getMouseTwoY() {
		return -mouseTwoY;
	}

	private double getMouseTwoDeltaX() {
		return mouseTwoDeltaX;
	}

	private double getMouseTwoDeltaY() {
		return -mouseTwoDeltaY;
	}

	private double getMouseOneDeltaX() {
		return mouseOneDeltaX;
	}

	private double getMouseOneDeltaY() {
		return -mouseOneDeltaY;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getTheta() {
		return theta;
	}

	// mouse 1 is left, mouse 2 is right
	private synchronized void computeOdometry(double dx1, double dy1, double dx2, double dy2) {
		// ticks to meters
		dx1 /= TICKS_PER_METER;
		dy1 /= TICKS_PER_METER;
		dx2 /= TICKS_PER_METER;
		dy2 /= TICKS_PER_METER;

		// mouse frame to robot start position frame
		// X axis of the robot is the Y axis of the mouse
		double dx1Robot =  dy1;
		double dy1Robot = -dx1;
		double dx2Robot =  dy2;
		double dy2Robot = -dx2;

		// useful definitions
		double yAvg  = (dy2Robot + dy1Robot) / 2;
		double xAvg  = (dx2Robot + dx1Robot) / 2;
		double xDiff =  dx2Robot - dx1Robot;
		double dTheta = xDiff / DISTANCE_BETWEEN_SENSORS;
		double thetaTravel = theta + dTheta / 2;

		// compute new odometry
		double xNew = x + Math.cos(thetaTravel) * xAvg - Math.sin(thetaTravel) * yAvg;
		double yNew = y + Math.sin(thetaTravel) * xAvg + Math.cos(thetaTravel) * yAvg;
		double thetaNew = theta + dTheta;

		// update new odometry
		x = xNew;
		y = yNew;
		theta = thetaNew;

		System.err.println("updating odometry: " + x + " " + y + " " + theta);
	}

	private synchronized void publishOdometry() {
		double robotX = x + Math.cos(theta) * SENSOR_Y_OFFSET;
		double robotY = y + Math.sin(theta) * SENSOR_Y_OFFSET;
		double robotTheta = theta;

		System.err.println("publishing odometry: " + robotX + " " + robotY + " " + robotTheta);
		msg.x = robotX;
		msg.y = robotY;
		msg.theta = robotTheta;
		pub.publish(msg);
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("/rss/TwoMice");
	}

	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
	}
}
