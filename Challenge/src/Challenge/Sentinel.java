package Challenge;

// some imports that may or may not be needed are commented out

/*
import java.awt.geom.*;
import java.awt.Color;
import java.io.IOException;
*/
import java.text.ParseException;
import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
/*
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
*/
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
/*
import org.ros.node.parameter.ParameterTree;
*/

/*
import LocalNavigation.Mat;
*/

public class Sentinel implements NodeMain {

	private Node thisNode;

	public static final String APPNAME = "Sentinel";

/*
	protected Grid grid;
	private String mapFile;
	protected PolygonMap map;
	private CSpace cspace;
*/

/*
	private Publisher<GUIRectMsg> rectPub;
	private Publisher<GUIPolyMsg> polyPub;
	private Publisher <GUIPointMsg> pointPub;
	private Publisher<GUIEraseMsg> erasePub;
	private Publisher<GUISegmentMsg> segPub;
*/

	private RobotController navigator;

	public Sentinel() {
	}

	public void onStart(Node node) {
		thisNode = node;

		System.err.println("SENTINEL INITIALIZING");
		navigator = new RobotController();
		System.err.println("ROBOT CONTROLLER CREATED");
		navigator.onStart(thisNode);
		System.err.println("ROBOT CONTROLLER INITIALIZED");
		navigator.addWaypoint(new Waypoint(1,   0, (short) 1));
		navigator.addWaypoint(new Waypoint(1,   1, (short) 1));
		//navigator.addWaypoint(new Waypoint(0.5, 1, (short) 1));
		//navigator.addWaypoint(new Waypoint(0,   0, (short) 1));
		//navigator.addWaypoint(new Waypoint(1,   0, (short) 1));
		//navigator.addWaypoint(new Waypoint(0,   0, (short) -1));
		System.err.println("WAYPOINTS ADDED");
		System.err.println("SENTINEL INITIALIZED");

		/*
		ParameterTree paramTree = node.newParameterTree();
		mapFile = paramTree.getString(node.resolveName("~/mapFileName"));
		try {
			//mapFile = "/home/rss-student/RSS-I-group/lab6/src/global-nav-maze-2011-basic.map";
			map = new PolygonMap(mapFile);
		} catch(IOException e) {
			System.out.println("IOException in PolygonMap");
			System.exit(0);
		} catch(ParseException e) {
			System.out.println("ParseException in PolygonMap");
			System.exit(0);
		}

		rectPub = node.newPublisher("/gui/Rect","lab6_msgs/GUIRectMsg");
		polyPub = node.newPublisher("/gui/Poly","lab6_msgs/GUIPolyMsg");
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		erasePub = node.newPublisher("/gui/Erase","lab5_msgs/GUIEraseMsg");
		segPub = node.newPublisher("/gui/Segment", "lab5_msgs/GUISegmentMsg");
		*/

		/*
		double halfWidth = 0.235;
		double fwd = 0.190;
		double bkwd = -0.290;

		double radius = 0.340;

		ArrayList<Mat> robotVerts = new ArrayList<Mat>();
		*/
		/*robotVerts.add(Mat.encodePoint(fwd,  -1 * halfWidth));
		robotVerts.add(Mat.encodePoint(bkwd, -1 * halfWidth));
		robotVerts.add(Mat.encodePoint(bkwd,      halfWidth));
		robotVerts.add(Mat.encodePoint(fwd,       halfWidth));*/
		/*robotVerts.add(Mat.encodePoint(     radius,      radius));
		robotVerts.add(Mat.encodePoint(-1 * radius,      radius));
		robotVerts.add(Mat.encodePoint(-1 * radius, -1 * radius));
		robotVerts.add(Mat.encodePoint(     radius, -1 * radius));*/
		/*
		for (double theta = 0; theta < 2 * Math.PI; theta += 0.05) {
			robotVerts.add(Mat.encodePoint(radius * Math.sin(theta), radius * Math.cos(theta)));
		}
		CSpace.Polygon robot = new CSpace.Polygon(robotVerts);

		cspace = new CSpace(robot, map);
		cspace.node = node;

		this.instanceMain();
		*/
	}


	/*
	public void instanceMain3D() {
		//TODO: Implement
		//Displays map, computes cspace and grid, displays grid and path, and initiates path following
		if(TESTCONVEXHULL) {
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			testConvexHull();
		} else {
			System.out.println("  map file: " + mapFile);

			// wait for the gui to come online.
			try {
				Thread.sleep(4000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			displayMap();

			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			int nCells = 150;
			boolean[][][] occupancyGrid3D = new boolean[nCells][nCells][8];
			for (int t = 0; t < 8; t++) {
				double halfWidth = 0.235;
				double fwd = 0.190;
				double bkwd = -0.290;

				double radius = 0.340;
				ArrayList<Mat> robotVerts = new ArrayList<Mat>();
				robotVerts.add(Mat.encodePoint(fwd,  -1 * halfWidth));
				robotVerts.add(Mat.encodePoint(bkwd, -1 * halfWidth));
				robotVerts.add(Mat.encodePoint(bkwd,      halfWidth));
				robotVerts.add(Mat.encodePoint(fwd,       halfWidth));
				CSpace.Polygon robot = new CSpace.Polygon(robotVerts);
				robot = CSpace.Polygon.mul(Mat.rotation(t * Math.PI / 4), robot);

				cspace = new CSpace(robot, map);
				cspace.node = thisNode;

				displayCSpace();

				boolean[][] occupancyGrid = cspace.getOccupancyGrid(nCells);
				displayCSpace();

				for (int i = nCells - 1; i >= 0; i--) {
					for (int j = 0; j < nCells; j++) {
						if (occupancyGrid[j][i]) {
							System.err.print(".");
							occupancyGrid3D[j][i][t] = true;
						} else {
							System.err.print("#");
							occupancyGrid3D[j][i][t] = false;
						}
					}
					System.err.println("");
				}
			}

			Rectangle2D.Double worldRect = map.getWorldRect();
			float mazeSize = Math.max((float)worldRect.getMaxX() - (float)worldRect.getMinX(), (float)worldRect.getMaxY() - (float)worldRect.getMinY());

			float startX = (float)map.getRobotStart().getX();
			float startY = (float)map.getRobotStart().getY();
			float endX = (float)map.getRobotGoal().getX();
			float endY = (float)map.getRobotGoal().getY();

			int startXi = Math.round((startX - (float)worldRect.getMinX()) / mazeSize * nCells);
			int startYi = Math.round((startY - (float)worldRect.getMinY()) / mazeSize * nCells);
			int endXi = Math.round((endX - (float)worldRect.getMinX()) / mazeSize * nCells);
			int endYi = Math.round((endY - (float)worldRect.getMinY()) / mazeSize * nCells);

			System.err.println(startXi);
			System.err.println(startYi);
			System.err.println(endXi);
			System.err.println(endYi);

			ArrayList<int[]> intWaypoints = new ArrayList<int[]>();
			try {
				System.err.println("START");
				intWaypoints = thirdclass.pathFind3D(occupancyGrid3D, new int[] {startXi, startYi, 0}, new int[] {endXi, endYi, 0});
				System.err.println("END");
			} catch (Exception e) {
				e.printStackTrace();
				System.exit(1);
			}

			for (int[] pt : intWaypoints) {
				System.err.println("(" + pt[0] + ", " + pt[1] + ")    ");
			}

			displayPath(intWaypoints);

			*/
			/*intWaypoints = new ArrayList<int[]>();
			intWaypoints.add(new int[] {0, 0});
			intWaypoints.add(new int[] {1, 0});
			intWaypoints.add(new int[] {2, 0});
			intWaypoints.add(new int[] {2, 1});
			intWaypoints.add(new int[] {2, 2});
			intWaypoints.add(new int[] {3, 3});
			intWaypoints.add(new int[] {4, 4});
			intWaypoints.add(new int[] {5, 5});
			intWaypoints.add(new int[] {4, 6});
			intWaypoints.add(new int[] {3, 7});
			intWaypoints.add(new int[] {2, 8});
			intWaypoints.add(new int[] {50, 51});
			intWaypoints.add(new int[] {50, 52});
			intWaypoints.add(new int[] {50, 53});
			intWaypoints.add(new int[] {50, 54});
			intWaypoints.add(new int[] {50, 55});
			intWaypoints.add(new int[] {50, 56});
			intWaypoints.add(new int[] {50, 57});
			intWaypoints.add(new int[] {50, 58});*/
			/*

			int i = 1;
			int dx1, dy1, dx2, dy2;
			while (i + 1 < intWaypoints.size()) {
				dx1 = intWaypoints.get(i)[0] - intWaypoints.get(i - 1)[0];
				dy1 = intWaypoints.get(i)[1] - intWaypoints.get(i - 1)[1];

				dx2 = intWaypoints.get(i + 1)[0] - intWaypoints.get(i)[0];
				dy2 = intWaypoints.get(i + 1)[1] - intWaypoints.get(i)[1];

				if (Math.abs(dx1) > 5 || Math.abs(dy1) > 5) {
					i++;
				} else {
					while (dx1 > 1) {
						dx1--;
					}
					while (dy1 > 1) {
					dy1--;
				}
				while (dx1 < -1) {
					dx1++;
				}
				while (dy1 < -1) {
					dy1++;
				}
				while (dx2 > 1) {
					dx2--;
				}
				while (dy2 > 1) {
					dy2--;
				}
				while (dx2 < -1) {
					dx2++;
				}
				while (dy2 < -1) {
					dy2++;
				}

				if (dx1 == dx2 && dy1 == dy2) {
					intWaypoints.remove(i);
				} else {
					i++;
				}
			}
		}

		RobotController navigator = new RobotController();
		navigator.onStart(thisNode);
		for (int[] pt : intWaypoints) {
			System.err.print("(" + pt[0] + ", " + pt[1] + ")    ");
			System.err.print((float)pt[0]);
			System.err.print(" ");
			System.err.print((float)pt[0] * mazeSize);
			System.err.print(" ");
			System.err.print((float)pt[0] * mazeSize / (float)nCells);
			System.err.print(" ");
			System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2);
			System.err.print(" ");
			System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX());
			System.err.print(" ");
			System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX() - startX);
			System.err.print(" ");
			float wpX = (float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX() - startX;
			float wpY = (float)pt[1] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinY() - startY;
			System.err.println("(" + wpX + ", " + wpY + ")");
			navigator.addWaypoint(new Waypoint(wpX, wpY, (short) 1));
		}
		navigator.addWaypoint(new Waypoint(endX, endY, (short) 1));
		}
	}
	*/

	/*
	public void instanceMain() {
		//TODO: Implement
		//Displays map, computes cspace and grid, displays grid and path, and initiates path following
		if(TESTCONVEXHULL) {
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			testConvexHull();
	
		} else if(MAPONLY) {
			System.out.println("  map file: " + mapFile);

			// wait for the gui to come online.
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			displayMap();
		} else {

			System.out.println("  map file: " + mapFile);

			// wait for the gui to come online.
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			displayMap();

			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			displayCSpace();

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			int nCells = 60;
			boolean[][] occupancyGrid = cspace.getOccupancyGrid(nCells);
			displayCSpace();

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			for (int i = nCells - 1; i >= 0; i--) {
				for (int j = 0; j < nCells; j++) {
					if (occupancyGrid[j][i]) {
						System.err.print(".");
					} else {
						System.err.print("#");
					}
				}
				System.err.println("");
			}

			Rectangle2D.Double worldRect = map.getWorldRect();
			float mazeSize = Math.max((float)worldRect.getMaxX() - (float)worldRect.getMinX(), (float)worldRect.getMaxY() - (float)worldRect.getMinY());

			float startX = (float)map.getRobotStart().getX();
			float startY = (float)map.getRobotStart().getY();
			float endX = (float)map.getRobotGoal().getX();
			float endY = (float)map.getRobotGoal().getY();

			int startXi = Math.round((startX - (float)worldRect.getMinX()) / mazeSize * nCells);
			int startYi = Math.round((startY - (float)worldRect.getMinY()) / mazeSize * nCells);
			int endXi = Math.round((endX - (float)worldRect.getMinX()) / mazeSize * nCells);
			int endYi = Math.round((endY - (float)worldRect.getMinY()) / mazeSize * nCells);

			System.err.println(startXi);
			System.err.println(startYi);
			System.err.println(endXi);
			System.err.println(endYi);

			ArrayList<int[]> intWaypoints = new ArrayList<int[]>();
			try {
				intWaypoints = thirdclass.pathFind(occupancyGrid, new int[] {startXi, startYi}, new int[] {endXi, endYi});
			} catch (Exception e) {
				e.printStackTrace();
				System.exit(1);
			}
	
			for (int[] pt : intWaypoints) {
				System.err.println("(" + pt[0] + ", " + pt[1] + ")    ");
			}
	
			displayPath(intWaypoints);

			*/
			/*intWaypoints = new ArrayList<int[]>();
			intWaypoints.add(new int[] {0, 0});
			intWaypoints.add(new int[] {1, 0});
			intWaypoints.add(new int[] {2, 0});
			intWaypoints.add(new int[] {2, 1});
			intWaypoints.add(new int[] {2, 2});
			intWaypoints.add(new int[] {3, 3});
			intWaypoints.add(new int[] {4, 4});
			intWaypoints.add(new int[] {5, 5});
			intWaypoints.add(new int[] {4, 6});
			intWaypoints.add(new int[] {3, 7});
			intWaypoints.add(new int[] {2, 8});
			intWaypoints.add(new int[] {50, 51});
			intWaypoints.add(new int[] {50, 52});
			intWaypoints.add(new int[] {50, 53});
			intWaypoints.add(new int[] {50, 54});
			intWaypoints.add(new int[] {50, 55});
			intWaypoints.add(new int[] {50, 56});
			intWaypoints.add(new int[] {50, 57});
			intWaypoints.add(new int[] {50, 58});*/
			/*

			int i = 1;
			int dx1, dy1, dx2, dy2;
			while (i + 1 < intWaypoints.size()) {
				dx1 = intWaypoints.get(i)[0] - intWaypoints.get(i - 1)[0];
				dy1 = intWaypoints.get(i)[1] - intWaypoints.get(i - 1)[1];

				dx2 = intWaypoints.get(i + 1)[0] - intWaypoints.get(i)[0];
				dy2 = intWaypoints.get(i + 1)[1] - intWaypoints.get(i)[1];

				if (Math.abs(dx1) > 5 || Math.abs(dy1) > 5) {
					i++;
				} else {
					while (dx1 > 1) {
						dx1--;
					}
					while (dy1 > 1) {
						dy1--;
					}
					while (dx1 < -1) {
						dx1++;
					}
					while (dy1 < -1) {
						dy1++;
					}
					while (dx2 > 1) {
						dx2--;
					}
					while (dy2 > 1) {
						dy2--;
					}
					while (dx2 < -1) {
						dx2++;
					}
					while (dy2 < -1) {
						dy2++;
					}

					if (dx1 == dx2 && dy1 == dy2) {
						intWaypoints.remove(i);
					} else {
						i++;
					}
				}
			}

			RobotController navigator = new RobotController();
			navigator.onStart(thisNode);
			for (int[] pt : intWaypoints) {
				System.err.print("(" + pt[0] + ", " + pt[1] + ")    ");
				System.err.print((float)pt[0]);
				System.err.print(" ");
				System.err.print((float)pt[0] * mazeSize);
				System.err.print(" ");
				System.err.print((float)pt[0] * mazeSize / (float)nCells);
				System.err.print(" ");
				System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2);
				System.err.print(" ");
				System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX());
				System.err.print(" ");
				System.err.print((float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX() - startX);
				System.err.print(" ");
				float wpX = (float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinX() - startX;
				float wpY = (float)pt[1] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)worldRect.getMinY() - startY;
				System.err.println("(" + wpX + ", " + wpY + ")");
				navigator.addWaypoint(new Waypoint(wpX, wpY, (short) 1));
			}
			navigator.addWaypoint(new Waypoint(endX, endY, (short) 1));
		}
	}
	*/

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

	/*
	public void displayMap() {
		erasePub.publish(new GUIEraseMsg());

		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		GUIRectMsg rectMsg = new GUIRectMsg();
		fillRectMsg(rectMsg, map.getWorldRect(), new Color(0,0,0), false);
		rectPub.publish(rectMsg);
		GUIPolyMsg polyMsg = new GUIPolyMsg();
		for (PolygonObstacle obstacle : map.getObstacles()){
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			polyMsg = new GUIPolyMsg();
			fillPolyMsg(polyMsg, obstacle, MapGUI.makeRandomColor(), true, true);
			polyPub.publish(polyMsg);
		}
		GUIPointMsg pointMsg = new GUIPointMsg();
		fillPointMsg(pointMsg, map.getRobotStart(), new Color(255,0,0), 1);
		pointPub.publish(pointMsg);
		fillPointMsg(pointMsg, map.getRobotGoal(), new Color(0,255,0), 1);
		pointPub.publish(pointMsg);
	}
	*/

	/*
	public void fillPointMsg(GUIPointMsg msg, java.awt.geom.Point2D.Double point, java.awt.Color color, long shape) {
		msg.x = (float) point.getX();
		msg.y = (float) point.getY();
		ColorMsg c = new ColorMsg();
		fillColor(c, color);
		msg.color = c;
		msg.shape = (int) shape;
	}

	public static void fillPolyMsg(GUIPolyMsg msg, PolygonObstacle obstacle, java.awt.Color c, boolean filled, boolean closed) {
		int i = 0;
		float[] xPoints = new float[obstacle.getVertices().size()];
		float[] yPoints = new float[obstacle.getVertices().size()];

		for (Point2D.Double p : obstacle.getVertices()) {
			xPoints[i] = (float) p.getX();
			yPoints[i] = (float) p.getY();
			i++;
		}

		msg.x = xPoints;
		msg.y = yPoints;
		msg.numVertices = i;

		ColorMsg color = new ColorMsg();
		fillColor(color, c);
		msg.c = color;
		if (filled) {
			msg.filled = 1;
		} else {
			msg.filled = 0;
		}

		if (closed) {
			msg.closed = 1;
		} else {
			msg.closed = 0;
		}
	}

	public static void fillRectMsg(GUIRectMsg msg, java.awt.geom.Rectangle2D.Double r, java.awt.Color c, boolean filled) {
		msg.x = (float) r.getX();
		msg.y = (float) r.getY();
		msg.width = (float) r.getWidth();
		msg.height = (float) r.getHeight();
		ColorMsg color = new ColorMsg();
		if(c == null) {
			c = new Color(0,0,0);
		}

		fillColor(color, c);
		msg.c = color;
		if (filled) {
			msg.filled = 1;
		} else {
			msg.filled = 0;
		}
	}

	public void fillSegmentMsg(GUISegmentMsg segmentMsg, java.awt.Color color, java.awt.geom.Point2D.Double start, java.awt.geom.Point2D.Double other) {
		segmentMsg.startX = start.getX();
		segmentMsg.startY = start.getY();
		segmentMsg.endX = other.getX();
		segmentMsg.endY = other.getY();
		ColorMsg c = new ColorMsg();
		fillColor(c, color);
		segmentMsg.color = c;
	}

	public static void fillColor(ColorMsg color2, java.awt.Color color) {
		color2.r = color.getRed();
		color2.g = color.getGreen();
		color2.b = color.getBlue();
	}

	public void displayGrid() {
		grid = new Grid(map.getWorldRect(), GRID_RESOLUTION);
	}
	*/

	/*
	public void displayCSpace() {
		GUIPolyMsg polyMsg = new GUIPolyMsg();
		for (CSpace.Polygon obstacle : cspace.getCSObstacles()) {
			polyMsg = new GUIPolyMsg();
			PolygonObstacle POobstacle = new PolygonObstacle();
			for (Mat vertex : obstacle.vertices) {
				double[] v = Mat.decodePoint(vertex);
				POobstacle.addVertex(v[0], v[1]);
			}
			fillPolyMsg(polyMsg, POobstacle, MapGUI.makeRandomColor(), true, true);
			polyPub.publish(polyMsg);
		}
	}

	public void displayCSpace(double theta) {
		GUIPolyMsg polyMsg = new GUIPolyMsg();
		for (CSpace.Polygon obstacle : cspace.getThetaObstacles(theta, Math.PI / 10)) {
			polyMsg = new GUIPolyMsg();
			PolygonObstacle POobstacle = new PolygonObstacle();
			for (Mat vertex : obstacle.vertices) {
				double[] v = Mat.decodePoint(vertex);
				POobstacle.addVertex(v[0], v[1]);
			}
			fillPolyMsg(polyMsg, POobstacle, MapGUI.makeRandomColor(), true, true);
			polyPub.publish(polyMsg);
		}
	}

	public void displayVisibilityGraph() {
		//TODO: Implement
	}

	public void displayPath(ArrayList<int[]> pts) {
		GUISegmentMsg segMsg = new GUISegmentMsg();
	
		for (int i=0; i<pts.size()-2; i++) {
			fillSegmentMsg(segMsg, new Color(0,0,0), new Point2D.Double(pts.get(i)[0], pts.get(i)[1]), new Point2D.Double(pts.get(i+1)[0], pts.get(i+1)[1]));
			segPub.publish(segMsg);
			try{
				Thread.sleep(100);
			} catch(InterruptedException e) {}
		}
	}

	public void testConvexHull() {
		erasePub.publish(new GUIEraseMsg());
		GUIPointMsg testPtMsg = new GUIPointMsg();
		GUIPolyMsg polyMsg = new GUIPolyMsg();

		ArrayList<Point2D.Double> pts = new ArrayList<Point2D.Double>();

		for(int i=0;i<10;i++) {
			pts.add(new Point2D.Double(3*Math.random(), 3*Math.random()));
			fillPointMsg(testPtMsg,pts.get(i),new Color(255,0,0),1);
			pointPub.publish(testPtMsg);
			try{
				Thread.sleep(100);
			} catch(InterruptedException e) {};
			}
			PolygonObstacle obs = GeomUtils.convexHull(pts);
		fillPolyMsg(polyMsg, obs, new Color(0,0,0),false, true);
		polyPub.publish(polyMsg);	
	}

	public void handle(OdometryMsg msg) {
		//TODO: Implement
	}

	public void handle(BumpMsg msg) {
		//TODO: Implement
	}
	*/
}
