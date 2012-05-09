package Challenge;

import LocalNavigation.Mat;
import GlobalNavigation.PolygonMap;

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

public class Sentinel implements NodeMain {

	private Node thisNode;
	public static final String APPNAME = "Sentinel";

	private RobotController navigator;

	private PolygonMap map;

	public Sentinel() {
	}

	public void onStart(Node node) {
		thisNode = node;

		System.err.println("SENTINEL INITIALIZING");
		navigator = new RobotController();
		System.err.println("ROBOT CONTROLLER CREATED");
		navigator.onStart(thisNode);
		System.err.println("ROBOT CONTROLLER INITIALIZED");

		/*
		double f2m = .3048;
		navigator.addWaypoint(new Waypoint(2   * f2m,  0   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(3   * f2m, -2   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(6   * f2m, -2   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(6.5 * f2m, -3.5 * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(7.5 * f2m, -4.5 * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(9   * f2m, -4.5 * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(7.5 * f2m, -3.5 * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(6.5 * f2m, -3.5 * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(6   * f2m, -1   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(2   * f2m,  0   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(0   * f2m,  0   * f2m, (short) 1));
		navigator.addWaypoint(new Waypoint(2   * f2m,  0   * f2m, (short)-1));

		System.err.println("WAYPOINTS ADDED");
		*/

		String mapFile = "/home/rss-student/RSS-I-group/Challenge/src/challenge_2012.txt";
		try {
			map = new PolygonMap(mapFile);
		} catch(IOException e) {
			System.err.println("IOException in PolygonMap");
			System.exit(0);
		} catch(ParseException e) {
			System.err.println("ParseException in PolygonMap");
			System.exit(0);
		} catch(Exception e) {
			System.err.println("Exception in PolygonMap");
			System.exit(0);
		}

		ArrayList<Mat> robotVerts = new ArrayList<Mat>();
		double radius = 0.340;
		for (double theta = 0; theta < 2 * Math.PI; theta += 0.05) {
			robotVerts.add(Mat.encodePoint(radius * Math.sin(theta), radius * Math.cos(theta)));
		}
		CSpace.Polygon robot = new CSpace.Polygon(robotVerts);

		CSpace cspace = new CSpace(robot, map);

		ArrayList<Mat> goals = new ArrayList<Mat>();
		goals.add(Mat.encodePoint(1, 0));
		goals.add(Mat.encodePoint(0, 1));
		goals.add(Mat.encodePoint(3, 4));

		Mat legStart = Mat.encodePoint(0, 0);
		ArrayList<Waypoint> waypoints;
		for (Mat goal: goals) {
			waypoints = PotentialField.findWaypoints(cspace, legStart, goal);
			System.err.println("Waypoints computed");
			for (Waypoint waypoint: waypoints) {
				navigator.addWaypoint(waypoint);
			}
			System.err.println("Waypoints added");
			legStart = goal;
		}

		System.err.println("WAYPOINTS ADDED");

		System.err.println("SENTINEL INITIALIZED");
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
