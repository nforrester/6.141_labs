package GlobalNavigation;

public class Waypoint {
	
	private final double myX;
	private final double myY;
	private final short myDir;
	
	/**
	 * <p>Create a Waypoint object</p>
	 *
	 * @param x x
	 * @param y y
	 * @param direction The direction the robot should drive in. Forward is 1, reverse is -1.
	 */
	public Waypoint(double x, double y, short direction) {
		myX=x;
		myY=y;
		myDir=direction;
	}
	
	public double getX(){
		return myX;
	}
	
	public double getY(){
		return myY;
	}
	
	public short getDir(){
		return myDir;
	}
	

}
