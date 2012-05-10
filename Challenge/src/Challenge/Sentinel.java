package Challenge;

import LocalNavigation.Mat;

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

		double robotYDim      = 0.24;
		double robotXDimFront = 0.20;
		double robotXDimBack  = 0.33;
		ArrayList<Mat> robotVerts = new ArrayList<Mat>();
		robotVerts.add(Mat.encodePoint(robotXDimFront,  robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimFront, -robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimBack,  -robotYDim));
		robotVerts.add(Mat.encodePoint(robotXDimBack,   robotYDim));
		CSpace.Polygon robot = new CSpace.Polygon(robotVerts);

		ArrayList<Mat> cRobotVerts = new ArrayList<Mat>();
		double radius = 0.340;
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

		// TESTING START (switch)
		//navigator = new RobotController(0, 0, cspace);
		navigator = new RobotController(map.robotStart.getX(), map.robotStart.getY(), cspace);
		// TESTING END
		System.err.println("ROBOT CONTROLLER CREATED");
		navigator.onStart(thisNode);
		System.err.println("ROBOT CONTROLLER INITIALIZED");
		Mat legStart = Mat.encodePoint(map.robotStart.getX(), map.robotStart.getY());
		ArrayList<Waypoint> waypoints;
		for (Mat goal: goals) {
			try {
				System.err.println("hello world!");
				waypoints = RapRandTree.findWaypoints(cspace, legStart, goal);
				System.err.println("Waypoints computed:");
				for (Waypoint waypoint: waypoints) {
					System.err.println("(waypoint " + waypoint.getX() + " " + waypoint.getY() + ")");
				}
				waypoints = trimWaypoints(waypoints);
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
	}

	private ArrayList<Waypoint> trimWaypoints(ArrayList<Waypoint> waypoints) {
		for (int legStart = 0; legStart < waypoints.size() - 2; legStart++) {
			Mat legStartPoint = Mat.encodePoint(waypoints.get(legStart).getX(), waypoints.get(legStart).getY());
			Mat legEndPoint = Mat.encodePoint(waypoints.get(legStart + 2).getX(), waypoints.get(legStart + 2).getY());
			if (cspace.lineSegInCSpace(legStartPoint, legEndPoint)) {
				waypoints.remove(legStart + 1);
			} else {
				legStart++;
			}
		}
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
