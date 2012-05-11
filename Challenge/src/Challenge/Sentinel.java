package Challenge;

import LocalNavigation.Mat;

import Challenge.PhaseTwo.PhaseTwoController;

import java.text.ParseException;
import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.io.IOException;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

public class Sentinel implements NodeMain {

	private Node thisNode;
	public static final String APPNAME = "Sentinel";

	private RobotController navigator;

	private GrandChallengeMap map;
	private CSpace cspace;

	public Sentinel() {
	}

	public void onStart(Node node) {
		thisNode = node;

		System.err.println("SENTINEL INITIALIZING");

		String mapFile = "/home/rss-student/gitrss/Challenge/src/construction_map_2012.txt";
		try {
			map = GrandChallengeMap.parseFile(mapFile);
		} catch(IOException e) {
			System.err.println("IOException in GrandChallengeMap");
			System.exit(1);
		} catch(ParseException e) {
			System.err.println("ParseException in GrandChallengeMap");
			System.exit(1);
		} catch(Exception e) {
			System.err.println("Exception in GrandChallengeMap");
			System.exit(1);
		}

		double robotYDim      = 0.27;
		double robotXDimFront = 0.23;
		double robotXDimBack  = 0.36;
		ArrayList<Mat> robotVerts = new ArrayList<Mat>();
		robotVerts.add(Mat.encodePoint(robotXDimFront,  robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimFront, -robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimBack,  -robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimBack,   robotYDim));
		CSpace.Polygon robot = new CSpace.Polygon(robotVerts);

		ArrayList<Mat> cRobotVerts = new ArrayList<Mat>();
		double radius = 0.37;
		for (double theta = 0; theta < 2 * Math.PI; theta += 0.05) {
			cRobotVerts.add(Mat.encodePoint(radius * Math.sin(theta), radius * Math.cos(theta)));
		}
		CSpace.Polygon cRobot = new CSpace.Polygon(cRobotVerts);

		cspace = new CSpace(robot, cRobot, map);

		ArrayList<Mat> goals = new ArrayList<Mat>();
		for (ConstructionObject block: map.getConstructionObjects()) {
			Point2D.Double pos = block.getPosition();
			goals.add(Mat.encodePoint(pos.getX(), pos.getY()));
		}

		// reject blocks north of the L obstacle
		int i = 0;
		while (i < goals.size()) {
			double pt[] = Mat.decodePoint(goals.get(i));
			if (pt[1] > 1.67) {
				goals.remove(i);
			} else {
				i++;
			}
		}

		// bubble sort the blocks from west to east
		boolean sorted;
		do {
			sorted = true;
			for (i = 0; i < goals.size() - 1; i++) {
				double pt0[] = Mat.decodePoint(goals.get(i));
				double pt1[] = Mat.decodePoint(goals.get(i + 1));
				if (pt1[0] < pt0[0]) {
					goals.set(i, Mat.encodePoint(pt1[0], pt1[1]));
					goals.set(i + 1, Mat.encodePoint(pt0[0], pt0[1]));
					sorted = false;
				}
			}
		} while (!sorted);

		// add goals that will bring it to a nice pose to build in
		goals.add(Mat.encodePoint(2.5, 1));
		goals.add(Mat.encodePoint(4, 0.5));

		// TESTING START (switch)
		//navigator = new RobotController(0, 0, cspace);
		navigator = new RobotController(map.robotStart.getX(), map.robotStart.getY(), cspace);
		// TESTING END
		System.err.println("ROBOT CONTROLLER CREATED");
		navigator.onStart(thisNode);
		System.err.println("ROBOT CONTROLLER INITIALIZED");

		executeGoals(map.robotStart.getX(), map.robotStart.getY(), goals);

		// TESTING START (comment)
		/*
		navigator.addWaypoint(new Waypoint(1, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 1, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(0, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(0, 1, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 1, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(0, 0, (short) 1));
		navigator.addWaypoint(new Waypoint(1, 1, (short) 1));
		navigator.addWaypoint(new Waypoint(0, 1, (short) 1));
		navigator.addWaypoint(new Waypoint(0, 0, (short) 1));
		*/
		// TESTING END

		System.err.println("WAYPOINTS ADDED");

		System.err.println("SENTINEL INITIALIZED");

		Thread monitorThread = new Thread(new Runnable() {
				@Override
				public void run() {
					while (true) {
						if (navigator.getNumWaypoints() == 0) {
							// TESTING START (uncomment)
							PhaseTwoController phaseTwo = new PhaseTwoController();
							// TESTING END
							break;
						}
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {
							e.printStackTrace();
							System.err.println("Interrupted! Exiting!");
							System.exit(1);
						}
					}
					System.err.println("BUILDING THE TOWER!");
				}
			});
	}

	private void executeGoals(double startX, double startY, ArrayList<Mat> goals) {
		Mat legStart = Mat.encodePoint(startX, startY);
		ArrayList<Waypoint> waypoints;
		for (Mat goal: goals) {
			try {
				System.err.println("hello world!");
				waypoints = RapRandTree.findWaypoints(cspace, legStart, goal);
				System.err.println("Waypoints computed:");
				for (Waypoint waypoint: waypoints) {
					System.err.println("(waypoint " + waypoint.getX() + " " + waypoint.getY() + ")");
				}
				waypoints = trimWaypoints(legStart, waypoints);
				System.err.println("Waypoints trimmed:");
				for (Waypoint waypoint: waypoints) {
					// TESTING START (uncomment)
					navigator.addWaypoint(waypoint);
					// TESTING END
					System.err.println("(waypoint " + waypoint.getX() + " " + waypoint.getY() + ")");
				}
				System.err.println("Waypoints added");
				legStart = goal;
			} catch (Exception e) {
				System.err.println("Goal is impossible");
				e.printStackTrace();
			}
		}
	}

	private ArrayList<Waypoint> trimWaypoints(Mat start, ArrayList<Waypoint> waypoints) {
		double st[] = Mat.decodePoint(start);
		waypoints.add(0, new Waypoint(st[0], st[1], (short) 1));
		int legStart = 0;
		while (legStart < waypoints.size() - 2) {
			Mat legStartPoint = Mat.encodePoint(waypoints.get(legStart).getX(), waypoints.get(legStart).getY());
			Mat legEndPoint = Mat.encodePoint(waypoints.get(legStart + 2).getX(), waypoints.get(legStart + 2).getY());
			if (cspace.lineSegInCSpace(legStartPoint, legEndPoint)) {
				System.err.println("Trimming " + waypoints.get(legStart + 1).getX() + " " + waypoints.get(legStart + 1).getY());
				waypoints.remove(legStart + 1);
			} else {
				System.err.println("Can't trim " + waypoints.get(legStart + 1).getX() + " " + waypoints.get(legStart + 1).getY());
				legStart++;
			}
		}
		waypoints.remove(0);
		return waypoints;
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
		return new GraphName("rss/Sentinel");
	}
}
