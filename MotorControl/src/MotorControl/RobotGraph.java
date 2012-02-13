package MotorControl;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JFrame;
import javax.swing.JPanel;

/*
 *  //TODO: make poseHistory a fixed size queue, so that it doesn't kill 
 *  		performance if left running for a long time
 *  //TODO: zoom in/out around the actual center of the screen, not the origin
 *  //TODO: put in a follow robot mode, where the robot center is always at the
 *  		center of the screen
 */

/**
 * RobotGraph is a GUI that tracks odometry for the first few labs, before Carmen is
 * introduced.  It can be useful to have running for debugging purposes, especially in
 * Lab 3.
 * 
 * To zoom in/out: use the mouse wheel, or CONTROL+UP/CONTROL+DOWN
 * 
 * To move screen around: use the arrow keys, or click and drag anywhere on the graph
 * 
 * @author Ryan Young
 */
public class RobotGraph extends JFrame implements Observer {
	private static final long serialVersionUID = -1299466487663318439L;

	//TODO see if we can delete this variable -- i don't think I actually use it
	private int[] prev_ticks; // (left wheel, right wheel, make int[]
        private static final double WHEEL_RADIUS_IN_M = WheelVelocityController.WHEEL_RADIUS_IN_M;
        private static final double ENCODER_RESOLUTION = WheelVelocityController.ENCODER_RESOLUTION;
        private static final double GEAR_RATIO = WheelVelocityController.GEAR_RATIO;
        private static final double WHEEL_METERS_PER_TICK = WHEEL_RADIUS_IN_M * Math.PI * 2/(WheelVelocityController.TICKS_PER_REVOLUTION);
	private static final double WHEELBASE = .428;
	/**
	 * [x][y][theta]
	 */
	private double[] pose = {0, 0, 0};
	
	/**
	 * All of the past positions the robot has been in
	 * since startup
	 */
	private List<double[]> poseHistory = new ArrayList<double[]>();

	private PaintablePanel p;
	

	private static final int FRAME_HEIGHT = 500;
	private static final int FRAME_WIDTH = 500;
	private static final double MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH = 1.05;
	private static final double X_MAX_INITIAL = 4.0;
	private static final double X_MIN_INITIAL = -4.0;
	private static final double Y_MAX_INITIAL = 4.0;
	private static final double Y_MIN_INITIAL = -4.0;
	
	private double total_mag = 1;
	// all values in meters
	private double x_step = 1.0;
	private double y_step = 1.0;
	private double x_max = X_MAX_INITIAL;
	private double x_min = X_MIN_INITIAL;
	private double y_max= Y_MAX_INITIAL;
	private double y_min = Y_MIN_INITIAL;
	
	private MyMouseListener ml = new MyMouseListener();
	private class MyMouseListener implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener  {
		//TODO put in a thread to make this animation continuous
		private int[] start_drag = new int[2];
		public void mouseClicked(MouseEvent e){}
		public void mouseEntered(MouseEvent e){}
		public void mouseExited(MouseEvent e){}
		public void mouseReleased(MouseEvent e){}
		public void mouseMoved(MouseEvent e){}

		public void mousePressed(MouseEvent e)
		{
			start_drag[0] = e.getX();
			start_drag[1] = e.getY();
		}


		public void mouseDragged(MouseEvent e) {

			// we need to find the x and y translation
			int diff_x = e.getX() - start_drag[0];
			int diff_y = e.getY() - start_drag[1];
			double dx = -diff_x/(1.0*FRAME_WIDTH)*(x_max-x_min);
			x_min += dx; x_max += dx;
			double dy = +diff_y/(1.0*FRAME_HEIGHT)*(y_max-y_min);
			y_min += dy; y_max += dy;
			start_drag[0] = e.getX();
			start_drag[1] = e.getY();
			repaint();
		}
		
		public void mouseWheelMoved(MouseWheelEvent e)
		{			
			//TEST to zoom on center of screen.
			int notches = e.getWheelRotation();
			if (notches < 0) { // zoom out, mag gets smaller
				total_mag /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				x_max /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				x_min /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				y_max /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				y_min /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
			}
			else { // zoom in, mag gets bigger
				total_mag *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				x_max *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				x_min *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				y_max *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
				y_min *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
			}
			repaint();
		}
		public void keyPressed(KeyEvent e) {
			if (e.isControlDown()) {
				switch (e.getKeyCode()) {
				case KeyEvent.VK_DOWN:
					// zoom in, mag gets bigger
					total_mag *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					x_max *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					x_min *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					y_max *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					y_min *= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					break;
				case KeyEvent.VK_UP:
					// zoom out, mag gets smaller
					total_mag /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					x_max /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					x_min /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					y_max /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					y_min /= MAG_INCREMENT_PER_MOUSE_WHEEL_NOTCH;
					break;
				}
			}
			else {
				double dx = 0;
				double dy = 0;
				switch (e.getKeyCode()) {
				case KeyEvent.VK_RIGHT:
					dx = -.025*(x_max-x_min);		
					break;
				case KeyEvent.VK_DOWN:
					dy = .025*(y_max-y_min);
					break;
				case KeyEvent.VK_LEFT:
					dx = .025*(x_max-x_min);
					break;
				case KeyEvent.VK_UP:
					dy = -.025*(y_max-y_min);
					break;
				}
				x_max += dx;
				x_min += dx;
				y_max += dy;
				y_min += dy;
			}
			repaint();
		}
		public void keyReleased(KeyEvent e) {}
		public void keyTyped(KeyEvent e) {}		
	}

	/**
	 * <p>Width of the body (Y direction) in meters.</p>
	 **/
	public static final double BODY_WIDTH = 0.38;
	/**
	 * <p>Length of the body (X direction) in meters.</p>
	 **/
	public static final double BODY_LENGTH = 0.38;
	/**
	 * <p>Width of the wheel (Y direction) in meters.</p>
	 **/
	public static final double WHEEL_WIDTH = 0.02;
	/**
	 * <p>Length of the wheel (X direction) in meters.</p>
	 **/
	public static final double WHEEL_LENGTH = 0.12;
	/**
	 * <p>Y displacement of the left wheel rect relative to robot ctr (m).</p>
	 **/
	public static final double WHEEL_DISPLACEMENT = 0.21;
	/**
	 * <p>X displacement of the body rect relative to robot ctr (m).</p>
	 **/
	public static final double BODY_DISPLACEMENT = -0.09;
	/**
	 * <p>Wheel shape centered on origin in world frame (m).</p>
	 **/
	public static final Shape WHEEL_SHAPE =
		new RoundRectangle2D.Double(
	    -WHEEL_LENGTH/2.0, -WHEEL_WIDTH/2.0,
	    WHEEL_LENGTH, WHEEL_WIDTH,
	    0.005, 0.005);

	/**
	 * <p>Body shape centered on origin in world frame (m).</p>
	 **/
	public static final Shape BODY_SHAPE =
		new Rectangle2D.Double(
	    -BODY_LENGTH/2.0, -BODY_WIDTH/2.0,
	    BODY_LENGTH, BODY_WIDTH);

	/**
	 * <p>Aggregate shape of the robot at pose (0, 0, 0).</p>
	 **/
	public static final GeneralPath ROBOT_SHAPE = new GeneralPath();
	static {
		AffineTransform t = new AffineTransform();
	
		t.setToIdentity();
		t.translate(BODY_DISPLACEMENT, 0.0);
		ROBOT_SHAPE.append(t.createTransformedShape(BODY_SHAPE), false);
	    
		t.setToIdentity();
		t.translate(0.0, WHEEL_DISPLACEMENT);
		ROBOT_SHAPE.append(t.createTransformedShape(WHEEL_SHAPE), false);
	     
		t.setToIdentity();
		t.translate(0.0, -WHEEL_DISPLACEMENT);
		ROBOT_SHAPE.append(t.createTransformedShape(WHEEL_SHAPE), false);
	     
		ROBOT_SHAPE.append(new Line2D.Double(0.0, 0.0, 0.01, 0.0), false);
		ROBOT_SHAPE.append(new Line2D.Double(0.0, 0.0, 0.0, 0.01), false);
	}
	
	public RobotGraph() {		
		// visually bring up the frame
		setPreferredSize(new Dimension(501,532));
		
		setWidgets();
		poseHistory.add(new double[] {0,0,0});
		
		setResizable(false);
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		pack();
		setVisible(true);
	}
	
	private void setWidgets() {
		p = new PaintablePanel();
		JPanel contentPane = new JPanel(new BorderLayout());
		contentPane.add(p, BorderLayout.CENTER);
		this.setContentPane(contentPane);
		this.addMouseWheelListener(ml);
		this.addMouseListener(ml);
		this.addMouseMotionListener(ml);
		this.addKeyListener(ml);
	}
	
	public class PaintablePanel extends JPanel {
		private static final long serialVersionUID = 6617333561144522727L;

		public PaintablePanel() {
			super(true);
		}
		public void paint(Graphics g2) {
			Graphics2D g = (Graphics2D) g2;
	        g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
	                RenderingHints.VALUE_STROKE_NORMALIZE);
	        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
	        		RenderingHints.VALUE_ANTIALIAS_ON);
	        g.setRenderingHint(RenderingHints.KEY_RENDERING,
	        		RenderingHints.VALUE_RENDER_QUALITY);
	        			
	        
//	        g.fillRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
	        g.setColor(Color.black);
	        
	        // transform to robot world coordinates
	        double xscale = FRAME_WIDTH/(x_max-x_min);
	        double yscale = FRAME_HEIGHT/(y_max-y_min);
			AffineTransform t = new AffineTransform();
			t.scale(1.0, -1.0);
			t.translate(FRAME_WIDTH/2-((x_min+x_max)/2.0*xscale), -FRAME_WIDTH/2-((y_min+y_max)/2.0*yscale));
			t.scale(xscale, yscale);
			g.setTransform(t);
						
			// set the stroke so that when it zooms in all the lines still look thin.
			g.setStroke(new BasicStroke((float)(1.0f*(x_max-x_min)/(FRAME_WIDTH*total_mag))));
			
			// erase the robot that used to be drawn
			int index = poseHistory.size()-2; // gets the pose right before the current one
			if (index >= 0 )
				paintPosition(	poseHistory.get(index)[0], poseHistory.get(index)[1],
								poseHistory.get(index)[2], g, true);
			// draw the grid
			drawGrid(g);
			drawAxes(g);
//			if (index >= 0 )
				paintRobotHistory(g);
			paintPosition(pose[0], pose[1], pose[2], g, false);
		}
		
		private void paintRobotHistory(Graphics2D g) {
			Color orig = g.getColor();
			g.setColor(Color.orange);
			Shape s;
			for (int i=0; i<poseHistory.size(); i++) {
				s = new Rectangle2D.Double(
						poseHistory.get(i)[0]-.01,
						poseHistory.get(i)[1]-.01,
						.02, .02);
				g.fill(s);
			}
			g.setColor(orig);
		}
		
		/**
		 * Paints the robot in the correct orientation on the map
		 * @param x x coordinate (in meters)
		 * @param y y coordinate (in meters)
		 * @param theta radial orientation (in radians)
		 * @param g the graphics object to paint on
		 * @param erase true if you want to erase the pose, false if you want to draw it
		 */
		private void paintPosition(double x, double y, double theta, Graphics2D g, boolean erase) {

		      AffineTransform t = new AffineTransform();
		      t.setToIdentity();
		      t.translate(x, y);
		      t.rotate(theta);

		      Color orig_color = g.getColor();
		      if (erase) {
		    	  g.setColor(new Color(238, 238, 238));
//		    	  g.setColor(Color.orange);
		    	  double w = ROBOT_SHAPE.getBounds2D().getWidth();
		    	  double h = ROBOT_SHAPE.getBounds2D().getHeight();
		    	  Shape s = new Rectangle2D.Double(
		    			  x-w,
		    			  y-h,
		    			  ROBOT_SHAPE.getBounds2D().getWidth()*2,
		    			  ROBOT_SHAPE.getBounds2D().getHeight()*2);
		    	  g.fill(s);
		      }
		      else
		    	  g.draw(t.createTransformedShape(ROBOT_SHAPE));
		      g.setColor(orig_color);
		}
		
		/**
		 * Draws the grid in green
		 */
		private void drawGrid(Graphics2D g) {
			Color orig = g.getColor();
			g.setColor(Color.GREEN);

			
			for (double i=x_min-2*x_step; i<=x_max+2*x_step; i+=x_step) {
				g.drawLine(	(int)(Math.round(i)), (int)(Math.round(y_min-2*y_step)),
							(int)(Math.round(i)), (int)(Math.round(y_max+2*y_step)));
			}
			for (double i=y_min-2*y_step; i<=y_max+2*y_step; i+=y_step) {
				g.drawLine(	(int)(Math.round(x_min-2*x_step)), (int)(Math.round(i)),
							(int)(Math.round(x_max+2*x_step)), (int)(Math.round(i)));
			}
			g.setColor(orig);
		}
		
		/**
		 * Draws the axes in black
		 */
		private void drawAxes(Graphics2D g) {
			Color orig = g.getColor();
			g.setColor(Color.BLACK);
			g.drawLine(	(int)(Math.round(x_min-2*x_step)), (int)(Math.round(0)),
						(int)(Math.round(x_max+2*x_step)), (int)(Math.round(0)));
			g.drawLine(	(int)(Math.round(0)), (int)(Math.round(y_min-2*y_step)),
						(int)(Math.round(0)), (int)(Math.round(y_max+2*y_step)));
			
			g.setColor(orig);			
		}
	}

	/**
	 * 
	 * @param ticks : each entry contains the number of encoder ticks since the
	 * last update 
	 */
	public void update(int[] ticks) {
		if (prev_ticks == null && (ticks[0]==0 && ticks[1]==0)) {
			prev_ticks = ticks;
		}
		else if (prev_ticks == null) {
			return;
		}
		if (ticks[0] == 0 && ticks[1] == 0) return; // we haven't moved at all
		
		double s_left = (ticks[0])*WHEEL_METERS_PER_TICK;
		double s_right = (ticks[1])*WHEEL_METERS_PER_TICK;
		double theta = (s_left - s_right)/WHEELBASE;
		pose[2] += -theta;
		pose[0] += (s_left+s_right)*Math.cos(pose[2])/2.0;
		pose[1] += (s_left+s_right)*Math.sin(pose[2])/2.0;
		System.out.println("pose : " + pose[0]);
		poseHistory.add(new double[] {pose[0], pose[1], pose[2]});
		prev_ticks[0] = ticks[0];
		prev_ticks[1] = ticks[1];
		repaint();
	}
	public void update(Observable arg0, Object arg1) {
		int[] ticks = (int[]) arg1;
		update(ticks);
	}
	
	public static void main(String[] args) {
		JFrame f = new RobotGraph();
		f.setPreferredSize(new Dimension(FRAME_WIDTH+1,FRAME_HEIGHT+32));
		f.setResizable(false);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.pack();
		f.setVisible(true);
	}
}
