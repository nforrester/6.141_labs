package GlobalNavigation;

import java.awt.geom.*;
import java.awt.Color;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;

import LocalNavigation.Mat;

public class GlobalNavigation implements NodeMain {

    public static final String APPNAME = "GlobalNavigation";

    public static final double FOLLOW_PATH_MAX_RV = .5;
    public static final double FOLLOW_PATH_RV_GAIN = .1;
    public static final double FOLLOW_PATH_TV = .2;
    public static final double GRID_RESOLUTION = .075;
    public static final double ROTATE_FIRST_THRESHOLD = .2;
    public static final double WP_THRESHOLD = .03;

    public static final int STOP = 0;
    public static final int GO = 1;

    protected Grid grid;
    private String mapFile;
    protected PolygonMap map;
    private CSpace cspace;

    private Publisher<GUIRectMsg> rectPub;
    private Publisher<GUIPolyMsg> polyPub;
    private Publisher <GUIPointMsg> pointPub;
    private Publisher<GUIEraseMsg> erasePub;

    public GlobalNavigation() {
    }


    public void onStart(Node node) {
	ParameterTree paramTree = node.newParameterTree();
	mapFile = paramTree.getString(node.resolveName("~/mapFileName"));
	try {
	    //	    mapFile = "/home/rss-student/RSS-I-group/lab6/src/global-nav-maze-2011-basic.map";
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

	double halfWidth = 0.235;
	double fwd = 0.190;
	double bkwd = 0.290;

	ArrayList<Mat> robotVerts = new ArrayList<Mat>();
	robotVerts.add(Mat.encodePoint(-1 * halfWidth, fwd));
	robotVerts.add(Mat.encodePoint(-1 * halfWidth, bkwd));
	robotVerts.add(Mat.encodePoint(     halfWidth, bkwd));
	robotVerts.add(Mat.encodePoint(     halfWidth, fwd));
	CSpace.Polygon robot = new CSpace.Polygon(robotVerts);

	cspace = new CSpace(robot, map);

	this.instanceMain();
    }

    
    public void instanceMain() {
	//TODO: Implement
	//Displays map, computes cspace and grid, displays grid and path, and initiates path following
	
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

	cspace.displayCSpace(0);
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
	return new GraphName("rss/GlobalNavigation");
    }

    public void displayMap() {
	erasePub.publish(new GUIEraseMsg());
	
	GUIRectMsg rectMsg = new GUIRectMsg();
	fillRectMsg(rectMsg, map.getWorldRect(), new Color(0,0,0), false);
	rectPub.publish(rectMsg);
	GUIPolyMsg polyMsg = new GUIPolyMsg();
	for (PolygonObstacle obstacle : map.getObstacles()){
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
	if (filled)
	    msg.filled = 1;
	else
	    msg.filled = 0;

	if (closed)
	    msg.closed = 1;
	else
	    msg.closed = 0;
    }

    public static void fillRectMsg(GUIRectMsg msg, java.awt.geom.Rectangle2D.Double r, java.awt.Color c, boolean filled) {
	msg.x = (float) r.getX();
	msg.y = (float) r.getY();
	msg.width = (float) r.getWidth();
	msg.height = (float) r.getHeight();
	ColorMsg color = new ColorMsg();
	if(c == null)
	    c = new Color(0,0,0);

	fillColor(color, c);
        msg.c = color;
	if (filled)
	    msg.filled = 1;
	else
	    msg.filled = 0;
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

    public void displayCSpace(double theta) {
	GUIPolyMsg polyMsg = new GUIPolyMsg();
	for (CSpace.Polygon obstacle : cspace.getThetaObstacles(theta, Math.PI / 10)) {
		polyMsg = new GUIPolyMsg();
		PolygonObstacle POobstacle;
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

    public void displayPath() {
	//TODO: Implement
    }

    public void testConvexHull() {
	//TODO: Implement
    }

    public void handle(OdometryMsg msg) {
	//TODO: Implement
    }
    
    public void handle(BumpMsg msg) {
	//TODO: Implement
    }

    
}
