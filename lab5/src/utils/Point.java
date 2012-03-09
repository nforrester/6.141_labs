package utils;

public class Point {
	public double x;
	public double y;
	public double theta;
	
	public Point(){
		this(0, 0, 0);
	}
	
	public Point(double x, double y){
		this(x, y, 0);
	}
	
	public Point(double x, double y, double theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
}
