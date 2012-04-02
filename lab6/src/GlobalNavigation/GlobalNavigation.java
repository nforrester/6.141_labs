package GlobalNavigation;

import java.awt.geom.*;
import java.awt.Color;
import java.io.IOException;
import java.text.ParseException;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;

public class GlobalNavigation implements NodeMain, Runnable {

    private String mapFileName;
    private PolygonMap map;

    private Publisher<GUIRectMsg> rectPub;
    private GUIRectMsg rectPlot;

    private Publisher<GUIPolyMsg> polyPub;
    private GUIPolyMsg polyPlot;

    private Publisher <GUIPointMsg> pointPub;
    private GUIPointMsg pointPlot;
    
    public GlobalNavigation() {
    }


    public void onStart(Node node) {
	ParameterTree paramTree = node.newParameterTree();
	mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));
	
	try {
	    map = new PolygonMap(mapFileName);
	} catch(IOException e) {
	    System.out.println("IOException in PolygonMap");
	    System.exit(0);
	} catch(ParseException e) {
	    System.out.println("ParseException in PolygonMap");
	    System.exit(0);
	}
	//initializes the rectangle publisher
	rectPub = node.newPublisher("/gui/Rect","lab6_msgs/GUIRectMsg");
	rectPlot = new GUIRectMsg();

	//initializes the polygon publisher
	polyPub = node.newPublisher("/gui/Poly","lab6_msgs/GUIPolyMsg");
	polyPlot = new GUIPolyMsg();

	//initializes the point publisher
	pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
	pointPlot = new GUIPointMsg();

	Thread t = new Thread(this);
	t.start();
    }
    
    @Override
    public void run() {
    }
     
    public void instanceMain(java.lang.String[] arg) {
	//TODO: Implement
	//Displays map, computes cspace and grid, displays grid and path, and initiates path following.
	displayMap();
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
	//display world rectangle
	Color rectC = new Color(0,0,0);
	fillRectMsg(rectPlot, map.getWorldRect(), rectC, false);
	rectPub.publish(rectPlot);

	//display obstacles
	Color polyC = new Color(0,0,0);
	for(PolygonObstacle o : map.getObstacles()) {
	    fillPolyMsg(polyPlot, o, polyC, true, true);
	    polyPub.publish(polyPlot);
	}

	//display start and end points
	Color startC = new Color(1,0,0);
	Color endC = new Color(0,1,0);
	fillPointMsg(pointPlot, map.getRobotStart(), startC, 0);
	pointPub.publish(pointPlot);
	fillPointMsg(pointPlot, map.getRobotGoal(), endC, 0);
	pointPub.publish(pointPlot);
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
	//TODO: Implement
    }

    public void displayCSpace() {
	//TODO: Implement
    }

    public void displayVisibilityGraph() {
	//TODO: Implement
    }

    public void displayPath() {
	//TODO: Implemet
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
