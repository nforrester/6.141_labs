package Grasping;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;

import javax.swing.JPanel;

@SuppressWarnings("serial")
public class ArmVisualizerPanel extends JPanel {
	private static final Point originPixelPoint = new Point();
	protected static final AffineTransform worldToPixelTransform = new AffineTransform();
	static {
		worldToPixelTransform.translate(300, 300);
		worldToPixelTransform.scale(500, -500);
		
		worldToPixelTransform.transform(new Point2D.Double(0,0), originPixelPoint);
	}
	
	private double armLength;
	private double gripperLength;
	
	private double shoulderOrientation;
	private double wristOrientation;
	
	private Point shoulderPixelPoint = new Point();
	private Point wristPixelPoint = new Point();
	private Point endEffectorPixelPoint = new Point();
	
	public ArmVisualizerPanel(){
		this(.24d,.185d);
	}
	public ArmVisualizerPanel (double upperarmLength,
					 		     double gripperLength) {
		
		this.armLength = upperarmLength;
		this.gripperLength = gripperLength;			
					
		shoulderOrientation = -Math.PI/4;
		wristOrientation = Math.PI/2;
		
		calculatePixelPoints();
		
		setPreferredSize(new Dimension(800, 400));
	  Thread pThread = new Thread(){
		public void run(){
			while(true){
				repaint();
				try{
					Thread.sleep(200);
				}catch(Exception e){
					e.printStackTrace();
				}
			}
		}
	};
	pThread.start();
	}
	
	@Override
	public void paint (Graphics g) {
		super.paint(g);
		
		g.setColor(Color.BLACK);
		
		// Draw origin
		g.drawOval(originPixelPoint.x-5, originPixelPoint.y-5, 10, 10);
		
		// Draw x-axis
		g.drawLine(0, originPixelPoint.y, Integer.MAX_VALUE, originPixelPoint.y);
		
		// Draw joints
		g.drawRect(shoulderPixelPoint.x-5, shoulderPixelPoint.y-5, 10, 10);
		g.drawOval(wristPixelPoint.x-5, wristPixelPoint.y-5, 10, 10);
		g.drawOval(endEffectorPixelPoint.x-5, endEffectorPixelPoint.y-5, 10, 10);
		
		// Draw arms
		g.drawLine(shoulderPixelPoint.x, shoulderPixelPoint.y,
				   wristPixelPoint.x, wristPixelPoint.y);
		g.drawLine(wristPixelPoint.x, wristPixelPoint.y,
				   endEffectorPixelPoint.x, endEffectorPixelPoint.y);
		
		// Draw angles
		String shoulderOrientationStr = Double.toString(shoulderOrientation);
		String wristOrientationStr = Double.toString(wristOrientation); 
		g.drawString(shoulderOrientationStr.substring(0, Math.min(5, shoulderOrientationStr.length())), 
					 shoulderPixelPoint.x+15, shoulderPixelPoint.y+15);
		g.drawString(wristOrientationStr.substring(0, Math.min(5, wristOrientationStr.length())),
				 	 wristPixelPoint.x+15, wristPixelPoint.y+15);
	}
	
	public void setShoulderOrientation (double shoulderOrientation) {
		this.shoulderOrientation = shoulderOrientation;
		//System.out.println("armVisPanel: this.shoulderOrientation"+this.shoulderOrientation);
		calculatePixelPoints();
		repaint();
	}
	
	public void setWristOrientation (double wristOrientation) {
		this.wristOrientation = wristOrientation;
		calculatePixelPoints();
		repaint();
	}
	
	private void calculatePixelPoints () {
		AffineTransform transform = new AffineTransform();
		
		//  Find shoulder point, given at origin
		Point2D.Double shoulderPoint = 
			//new Point2D.Double(Grasping.MOUNT_POINT.x, Grasping.MOUNT_POINT.y);
			new Point2D.Double(0.065, 0.28);
		transform.translate(shoulderPoint.x, shoulderPoint.y);
		
		//  Find wrist point
		Point2D.Double wristPoint = new Point2D.Double();
		transform.rotate(shoulderOrientation);
		transform.translate(armLength, 0);
		transform.transform(new Point2D.Double(0,0), wristPoint);
				
		//  Find end effector point
		Point2D.Double endEffectorPoint = new Point2D.Double();
		transform.rotate(wristOrientation);
		transform.translate(gripperLength, 0);
		transform.transform(new Point2D.Double(0,0), endEffectorPoint);
		
		// Transform world coordinates to pixel coordinates
		worldToPixelTransform.transform(shoulderPoint, shoulderPixelPoint);
		worldToPixelTransform.transform(wristPoint, wristPixelPoint);
		worldToPixelTransform.transform(endEffectorPoint, endEffectorPixelPoint);
	}

}
