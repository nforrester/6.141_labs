package LocalNavigation;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.*;
import org.ros.node.Node;

import VisualServo.VisionGUI;
import org.ros.node.topic.Subscriber;


/**
 * <p>Extends <code>VisualServo.VisionGUI</code> to display sonar-related data
 * (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding Carmen messages) have been added to draw
 * points, lines, and line segments.</p>
 *   
 * @author vona
 **/
public class SonarGUI extends VisionGUI {

  /**
   * <p>The application name.</p>
   **/
  public static final String APPNAME = "SonarGUI";

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_POINTS = 1<<3;

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_SEGMENTS = 1<<4;

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_LINE = 1<<5;

  /**
   * <p>Cross-shaped point.</p>
   **/
  public static final int X_POINT = 0;

  /**
   * <p>Box-shaped point.</p>
   **/
  public static final int O_POINT = 1;

  /**
   * <p>Line width of the lines making up a {@link SonarGUI.GUIPoint} in
   * pixels.</p>
   **/
  public static final float POINT_LINE_WIDTH = 0.5f;

  /**
   * <p>Line width of the lines making up a {@link SonarGUI.Segment} in
   * pixels.</p>
   **/
  public static final float SEGMENT_LINE_WIDTH = 1.0f;

  /**
   * <p>Line width of the lines making up a {@link SonarGUI.Line} in
   * pixels.</p>
   **/
  public static final float LINE_LINE_WIDTH = 0.5f;

  /**
   * <p>Default color for {@link SonarGUI.GUIPoint}s.</p>
   **/
  public static final Color DEFAULT_POINT_COLOR = Color.RED;

  /**
   * <p>Default color for {@link SonarGUI.Segment}s.</p>
   **/
  public static final Color DEFAULT_SEGMENT_COLOR = Color.RED;

  /**
   * <p>Default color for {@link SonarGUI.Line}s.</p>
   **/
  public static final Color DEFAULT_LINE_COLOR = Color.PINK;

  /**
   * <p>Radius of {@link SonarGUI.GUIPoint}s in meters.</p>
   **/
  public static final double POINT_RADIUS = 0.005;

  /**
   * <p>Whether to paint the points.</p>
   **/
  protected boolean pointsEnabled = true;

  /**
   * <p>Whether to paint the segments.</p>
   **/
  protected boolean segmentsEnabled = true;

  /**
   * <p>Whether to paint the line.</p>
   **/
  protected boolean lineEnabled = true;

  /**
   * <p>The current {@link SonarGUI.GUIPoint} color.</p>
   **/
  protected Color pointColor = dupColor(DEFAULT_POINT_COLOR);

  /**
   * <p>The current {@link SonarGUI.Segment} color.</p>
   **/
  protected Color segmentColor = dupColor(DEFAULT_SEGMENT_COLOR);

  /**
   * <p>A visual point.</p>
   **/
  protected class GUIPoint extends Glyph {

    /**
     * <p>Point color.</p>
     **/
    Color color;

    /**
     * <p>Point shapes.</p>
     **/
    Shape[] shape;

      int shapeType;

      public int hashCode() {
	  return 0;
      }

      @Override public boolean equals(Object other) {
	  GUIPoint another = (GUIPoint)other;
	  if ( shapeType != another.shapeType ) return false;
	  if ( !color.equals(another.color) ) return false;
	  switch ( shapeType ) {
	  case X_POINT: {
	      Line2D.Double a0 = (Line2D.Double)shape[0];
	      Line2D.Double a1 = (Line2D.Double)shape[1];
	      Line2D.Double b0 = (Line2D.Double)another.shape[0];
	      Line2D.Double b1 = (Line2D.Double)another.shape[1];
	      if ( a0.getX1() != b0.getX1() ||
		   a0.getX2() != b0.getX2() ||
		   a1.getX1() != a1.getX1() ||
		   a1.getX2() != a1.getX2() ) return false;
	      break;
	  } case O_POINT : {
		if (!shape[0].equals(another.shape[0])) return false;
		break;
	    }
	  default : { return false; }
	  }
	  return true;
      }

    /**
     * <p>Create a new GUI point.</p>
     **/
    GUIPoint(double x, double y, int shape, Color color) {

	this.shapeType = shape;

	this.color = dupColor(color);

      double xMin = x - POINT_RADIUS;
      double yMin = y - POINT_RADIUS;

      switch (shape) {
      case X_POINT: {

        double xMax = x + POINT_RADIUS;
        double yMax = y + POINT_RADIUS;

        this.shape = new Shape[2];
        this.shape[0] = new Line2D.Double(xMin, yMin, xMax, yMax);
        this.shape[1] = new Line2D.Double(xMin, yMax, xMax, yMin);

        break;

      } case O_POINT: {

        this.shape = new Shape[1];
        this.shape[0] = new Rectangle2D.Double(xMin, yMin, 
                                               2.0*POINT_RADIUS,
                                               2.0*POINT_RADIUS);
        break;
      }
      default: {
        this.shape = new Shape[0];
      }
      }
    }

    /**
     * <p>Paints the point.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {
      g2d.setColor(color);
      for (int i = 0; i < shape.length; i++) {
        g2d.draw(shape[i]);
      }
    }
  }

  /**
   * <p>A line segment.</p>
   **/
  protected class Segment extends Glyph {

    /**
     * <p>Segment color.</p>
     **/
    Color color;

    /**
     * <p>The segment.</p>
     **/
    Line2D.Double segment;

      public int hashCode() {
	  return 0;
      }

      @Override public boolean equals(Object other) {
	  Segment another = (Segment)other;
	  if ( !color.equals(another.color) ) return false;
	  return ( segment.getX1() == another.segment.getX1() && 
		   segment.getX2() == another.segment.getX2() && 
		   segment.getY1() == another.segment.getY1() && 
		   segment.getY2() == another.segment.getY2() );
      }

    /**
     * <p>Create a new segment.</p>
     **/
    Segment(double sx, double sy, double ex, double ey, Color color) {
      segment = new Line2D.Double(sx, sy, ex, ey);
      this.color = dupColor(color);
    }

    /**
     * <p>Paints the segment.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {
      g2d.setColor(color);
      g2d.draw(segment);
    }
  }

  /**
   * <p>A line.</p>
   *
   * <p>See {@link SonarGUI#setLine}.</p>
   **/
  protected class Line extends Glyph {

    /**
     * <p>Parameter "a".</p>
     **/
    double a = 0.0;

    /**
     * <p>Parameter "b".</p>
     **/
    double b = 0.0;

    /**
     * <p>Parameter "c".</p>
     **/
    double c = 0.0;

    /**
     * <p>Line color.</p>
     **/
    Color color = dupColor(DEFAULT_LINE_COLOR);

    /**
     * <p>Temp space.</p>
     **/
    double[] x = new double[2];

    /**
     * <p>Temp space.</p>
     **/
    double[] y = new double[2];

    /**
     * <p>Temp space.</p>
     **/
    Line2D.Double line = new Line2D.Double();

    /**
     * <p>Paints the line.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {

      // avoid NPE on init
      if (color == null)
        return;

      // disabled or invalid?
      if ((a == 0.0) && (b == 0.0))
        return;

      g2d.setColor(color);

      //current viewport dimensions in meters
      double width = ((double)getWidth())/scale;
      double height = ((double)getHeight())/scale;
      
      //current viewport bounds in meters
      double xMin = cx - width/2.0;
      double xMax = cx + width/2.0;
      
      double yMin = cy - height/2.0;
      double yMax = cy + height/2.0;

      //y values of intersections with left and right sides
      double yAtXmin = -(a*xMin+c)/b; // b = 0 results in Infinity
      double yAtXmax = -(a*xMax+c)/b; // b = 0 results in infinity

      //x values of intersections with bottom and top
      double xAtYmin = -(b*yMin+c)/a; // a = 0 results in infinity
      double xAtYmax = -(b*yMax+c)/a; // a = 0 results in infinity

      int next = 0;

      //intersect left side
      if ((next < 2) && (yAtXmin >= yMin) && (yAtXmin <= yMax)) {
        x[next] = xMin;
        y[next] = yAtXmin;
        next++;
      }
        
      //intersect right side
      if ((next < 2) && (yAtXmax >= yMin) && (yAtXmax <= yMax)) {
        x[next] = xMax;
        y[next] = yAtXmax;
        next++;
      }
        
      //intersect bottom
      if ((next < 2) && (xAtYmin >= xMin) && (xAtYmin <= xMax)) {
        x[next] = xAtYmin;
        y[next] = yMin;
        next++;
      }

      //intersect top
      if ((next < 2) && (xAtYmax >= xMin) && (xAtYmax <= xMax)) {
        x[next] = xAtYmax;
        y[next] = yMax;
        next++;
      }

      if (next < 2)
        return; //out of bounds

      line.x1 = x[0];
      line.x2 = x[1];

      line.y1 = y[0];
      line.y2 = y[1];

      g2d.draw(line);
    }
  }

  /**
   * <p>All the {@link SonarGUI.GUIPoint}s.</p>
   **/
  protected java.util.Set<GUIPoint> points =
  Collections.synchronizedSet(new HashSet<GUIPoint>());

  /**
   * <p>All the {@link SonarGUI.Segment}s.</p>
   **/
  protected java.util.Set<Segment> segments =
  Collections.synchronizedSet(new HashSet<Segment>());

  /**
   * <p>The one {@link SonarGUI.Line}.</p>
   **/
  protected Line line = new Line();

  /**
   * <p>Consruct a new SonarGUI.</p>
   *
   * <p>See <code>VisualServo.VisionGUI(int, double, double)</code>.</p>
   **/
  public SonarGUI(int poseSaveInterval, double maxTV, double maxRV) {
    super(poseSaveInterval, maxTV, maxRV);
  }

  /**
   * <p>See <code>VisualServo.VisionGUI(int)</code>.</p>
   **/
  public SonarGUI(int poseSaveInterval) {
    super(poseSaveInterval);
  }

  /**
   * <p>See <code>VisualServo.VisionGUI()</code>.</p>
   **/
  public SonarGUI() {
    super(60);
  }

  /**
   * <p>Make a copy of a color.</p>
   *
   * @param c the color to copy
   * @return an independent copy of c, or null if c was null
   **/
  public static Color dupColor(Color c) {

    if (c == null)
      return null;
    
    return new Color(c.getRed(), c.getGreen(), c.getBlue(), c.getAlpha());
  }

  /**
   * <p>Generate a random color.</p>
   *
   * @return a random color
   **/
  public static Color makeRandomColor() {
    return new Color((float) Math.random(),
                     (float) Math.random(),
                     (float) Math.random());
  }

  /**
   * {@inheritDoc}
   *
   * <p>Default impl returns {@link #APPNAME}.</p>
   *
   * @return the title for the GUI frame
   **/
  public String getAppName() {
    return APPNAME;
  }

  /**
   * <p>Add a point for display.</p>
   * 
   * @param x the point x position in world frame (m)
   * @param y the point y position in world frame (m)
   * @param shape {@link #X_POINT} or {@link #O_POINT}
   * @param color the point color or null to use current
   **/
  public void addPoint(double x, double y, int shape, Color color) {

    synchronized (points) {

      if (color != null)
        pointColor = dupColor(color);

      points.add(new GUIPoint(x, y, shape, pointColor));
    }

    repaint();
  }

  /**
   * <p>Convenience cover of {@link #addPoint(double, double, int, Color)},
   * always uses current color.</p>
   **/
  public void addPoint(double x, double y, int shape) {
    addPoint(x, y, shape, null);
  }

  /**
   * <p>Add a segment for display.</p>
   * 
   * @param sx the segment start x position in world frame (m)
   * @param sy the segment start y position in world frame (m)
   * @param ex the segment end x position in world frame (m)
   * @param ey the segment end y position in world frame (m)
   * @param color the segment color or null to use current
   **/
  public void addSegment(double sx, double sy, double ex, double ey,
                         Color color) {

    synchronized (segments) {
      
      if (color != null)
        segmentColor = dupColor(color);

      segments.add(new Segment(sx, sy, ex, ey, segmentColor));
    }

    repaint();
  }

  /**
   * <p>Convenience cover of {@link #addSegment(double, double, double, double,
   * Color)}, always uses current color.</p>
   **/
  public void addSegment(double sx, double sy, double ex, double ey) {
    addSegment(sx, sy, ex, ey, null);
  }

  /**
   * <p>Set the line coordinates.</p>
   *
   * <p>The line is represented by its coordinates (a, b, c) in
   * <pre>
   * ax + by + c = 0
   * </pre>
   * which defines the locus of points (x, y) on the line.</p>
   *
   * <p>Not all three coordinates may be zero (if they are, the line is
   * disabled).  This also implies that not both a and b can be zero, since if
   * they were and c were not zero then no points would satisfy the above
   * equation.</p>
   *
   * @param a the line a coord
   * @param b the line b coord
   * @param c the line c coord
   * @param color the line color or null to use current
   **/
  public void setLine(double a, double b, double c, Color color) {
    synchronized (line) {

      if (color != null)
        line.color = dupColor(color);

      line.a = a;
      line.b = b;
      line.c = c;
    }
    repaint();
  }
 
  /**
   * <p>Convenience cover of {@link #setLine(double, double, double, Color)},
   * always uses current color.</p>
   **/
  public void setLine(double a, double b, double c) {
    setLine(a, b, c, null);
  }

  /**
   * <p>Erase all previously plotted points.</p>
   **/
  public void erasePoints() {
    points.clear();
    repaint();
  }

  /**
   * <p>Erase all previously segments.</p>
   **/
  public void eraseSegments() {
    segments.clear();
    repaint();
  }

  /**
   * <p>Erase the line, if any.</p>
   **/
  public void eraseLine() {
    synchronized (line) {
      line.a = 0.0;
      line.b = 0.0;
      line.c = 0.0;
    }
    repaint();
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl {@link #paintPoints}, {@link #paintSegments}, {@link
   * #paintLine}, iff each is enabled.</p>
   **/
  protected void paintInWorldOverPosesHook(Graphics2D g2d) {

    if (pointsEnabled)
      paintPoints(g2d);
    
    if (segmentsEnabled)
      paintSegments(g2d);
    
    if (lineEnabled)
      paintLine(g2d);
  }

  /**
   * <p>Paint all {@link #points}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintPoints(Graphics2D g2d) {

    //avoid NPE on init
    if (points == null)
      return;

    setLineWidth(g2d, POINT_LINE_WIDTH);

    synchronized (points) {
      for (Iterator it = points.iterator(); it.hasNext(); )
        ((GUIPoint) it.next()).paint(g2d);
    }
  }

  /**
   * <p>Paint all {@link #segments}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintSegments(Graphics2D g2d) {

    //avoid NPE on init
    if (segments == null)
      return;

    setLineWidth(g2d, SEGMENT_LINE_WIDTH);

    synchronized (segments) {
      for (Iterator it = segments.iterator(); it.hasNext(); )
        ((Segment) it.next()).paint(g2d);
    }
  }

  /**
   * <p>Paint {@link #line}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintLine(Graphics2D g2d) {

    //avoid NPE on init
    if (line == null)
      return;

    setLineWidth(g2d, LINE_LINE_WIDTH);

    synchronized (line) {
      line.paint(g2d);
    }
  }

    public Subscriber<org.ros.message.lab5_msgs.GUILineMsg> guiLineSub;
    public Subscriber<org.ros.message.lab5_msgs.GUISegmentMsg> guiSegmentSub;
    public Subscriber<org.ros.message.lab5_msgs.GUIPointMsg> guiPointSub;
    public Subscriber<org.ros.message.lab5_msgs.GUIEraseMsg> guiEraseSub;

  /**
   * <p>See <code>VisualServo.VisionGUI.instanceMain()</code> and {@link
   * #mainPostHook}.</p>
   **/
  @Override
  public void onStart(Node node) {
      super.onStart(node);
      guiLineSub = node.newSubscriber("gui/Line", "lab5_msgs/GUILineMsg");
      guiSegmentSub = node.newSubscriber("gui/Segment", "lab5_msgs/GUISegmentMsg");
      guiPointSub = node.newSubscriber("gui/Point", "lab5_msgs/GUIPointMsg");
      guiEraseSub = node.newSubscriber("gui/Erase", "lab5_msgs/GUIEraseMsg");

      guiLineSub.addMessageListener(new LineMessageListener(this));
      guiSegmentSub.addMessageListener(new SegmentMessageListener(this));
      guiPointSub.addMessageListener(new PointMessageListener(this));
      guiEraseSub.addMessageListener(new EraseMessageListener(this));
  }


  /**
   * {@inheritDoc}
   *
   * <p>This impl tests the sonar graphics.</p>
   **/
  public void testGraphicsHook() throws InterruptedException {

    addPoint(0.5, 0.5, X_POINT, Color.GREEN);
    addPoint(-0.5, -0.5, O_POINT);

    for (int i = 0; i < 500; i++) {
      for (int j = 0; j < 10; j++) {
        addPoint(Math.random(), Math.random(),
                 (int) Math.round(Math.random()),
                 makeRandomColor());
      }
    }

    addSegment(0.0, 0.0, 1.0, 1.0, Color.BLUE);
    addSegment(-0.5, -0.5, -0.5, 0.0);
    
    for (int i = 0; i < 500; i++) {
      for (int j = 0; j < 10; j++) {
        addSegment(Math.random(), Math.random(),
                   Math.random(), Math.random(),
                   makeRandomColor());
      }
    }

    setLine(-1.0, -1.0, 0.5);
    Thread.sleep(1000);
    setLine(0.0, 0.0, 0.0);
    Thread.sleep(1000);
    setLine(1.0, 0.0, 1.0);
    Thread.sleep(1000);
    setLine(2.0, 2.0, 0.0);
    Thread.sleep(1000);
    setLine(-1.0, 0.0, 0.0);
    Thread.sleep(1000);
    setLine(0.0, 1.0, -0.5);
    Thread.sleep(1000);
    eraseLine();
  }
}
