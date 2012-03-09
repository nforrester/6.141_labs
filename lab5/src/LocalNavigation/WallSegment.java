package LocalNavigation;


import utils.Point;

/**
 * <p>Class for storing linear wall segments.  Stores the
 * start and end points of the segment, and the parameters
 * of the original linear regression fit<\p>
 */
public class WallSegment {
    // start and end points
    private Point startPoint;
    private Point endPoint;
    
    // used to define the linear fit: ax+by+c=0
    private double a;
    private double b;
    private double c;

    
    /**
     * <p>Constructs an empty Wall Segment<\p>
     */
    public WallSegment() {
        startPoint = new Point();
        endPoint = new Point();
    }

    
    /**
     * <p>Constructs a Wall Segment with specified
     * parameters.  Linear fit parameters correspond
     * to the line equation, ax+by+c=0<\p>
     *
     * @param start point of the segment
     * @param end point of the segment
     * @param the linear fit x scale parameter
     * @param the linear fit y scale parameter
     * @param the linear fit offset parameter
     */
    public WallSegment(Point startPoint, Point endPoint, double a,
                       double b, double c) {
        this();
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        
        this.a = a;
        this.b = b;
        this.c = c;
    }


    /**
     *
     */
    public double getA() {
        return a;
    }

    /**
     *
     */
    public double getB() {
        return b;
    }

    /**
     *
     */
    public double getC() {
        return c;
    }

    /**
     *
     */
    public Point getEndPoint() {
        return endPoint;
    }

    /**
     *
     */
    public Point getStartPoint() {
        return startPoint;
    }


    /**
     * Return the angle of this segment relative
     * to the x-axis.
     */
    public double getTheta() {
        double delta_x = endPoint.x - startPoint.x;
        double delta_y = endPoint.y - startPoint.y;
        return Math.atan2(delta_y, delta_x);
    }
    
}


