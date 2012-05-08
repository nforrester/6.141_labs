
public class KinectPixel {

	private final int row;
	private final int col;
	private final float x;
	private final float y;
	private final float z;
	private final int red;
	private final int green;
	private final int blue;
	private final float hue;
	private final float sat;
	private final float brt;
	
	/**
	 * Instantiates a new kinect pixel
	 * with the given parameters.
	 * 
	 * @param row
	 * @param col
	 * @param x
	 * @param y
	 * @param z
	 * @param red
	 * @param green
	 * @param blue
	 */
	public KinectPixel(int row,int col,float x,float y,float z,
					int red,int green,int blue){
		this.row = row;
		this.col = col;
		this.x =x;
		this.y =y;
		this.z =z;
		this.red = red;
		this.blue = blue;
		this.green = green;
		
		float[] hsb = java.awt.Color.RGBtoHSB(red, green, blue, null);
		hue = hsb[0];
		sat = hsb[1];
		brt = hsb[2];
	}
	
	/**
	 * Changes the color of 
	 * the given kinect pixel and 
	 * return a new Kinect pixel 
	 * with the same depths.
	 * This is most probably 
	 * used for debugging.
	 * @param new kinect pixel
	 * @param red
	 * @param green
	 * @param blue
	 * @return kinect pixel after
	 * 	color change
	 */
	public KinectPixel changeColor(int red,int green,int blue){
		KinectPixel newPixel = new KinectPixel(this.getRow(),this.getColumn(),this.getX(),
									this.getY(),this.getZ(),red,green,blue);
		return newPixel;
	}
	
	/**
	 * copies the kinect pixel
	 * @return a copy of the pixel
	 */
	public KinectPixel copyPixel(){
		return new KinectPixel(this.getRow(),this.getColumn(),this.getX(),
				this.getY(),this.getZ(),this.getRed(),this.getGreen(),this.getBlue());
	}
	
	/**
	 * Gets the row index of the pixel
	 * @return the row index of
	 * 			the pixel
	 */
	public int getRow(){
		return this.row;
	}
	
	/**
	 * Gets the column index
	 * of the pixel
	 * @return the col index of the pixel
	 */
	public int getColumn(){
		return this.col;
	}
	
	/**
	 * Gets the hue of the pixel
	 * @return the hue index of
	 * 			the pixel
	 */
	public float getHue(){
		return this.hue;
	}
	
	/**
	 * Gets the saturation of the pixel
	 * @return the saturation index of
	 * 			the pixel
	 */
	public float getSaturation(){
		return this.sat;
	}
	
	/**
	 * Gets the brighness of the pixel
	 * @return the brightness index of
	 * 			the pixel
	 */
	public float getBrightness(){
		return this.brt;
	}
	
	/**
	 * Gets the red value of
	 * the pixel 
	 * @return the red value 
	 * 			of the pixel
	 */
	public int getRed(){
		return this.red;
	}
	
	/**
	 * Gets the green value of 
	 * the pixel
	 * @return the green value of
	 * 			the pixel
	 */
	public int getGreen(){
		return this.green;
	}
	
	/**
	 * Gets the blue value of the
	 * pixel 
	 * @return the blue value 
	 * 			of the pixel
	 */
	public int getBlue(){
		return this.blue;
	}

	/**
	 * Gets the global X value
	 * of the pixel
	 * @return the global X 
	 * 			of the pixel
	 */
	public float getX(){
		return this.x;
	}
	
	/**
	 * Gets the global Y value
	 * of the pixel
	 * @return the global Y
	 * 		of the pixel
	 */
	public float getY(){
		return this.y;
	}
	
	/**
	 * Gets the global Z value
	 * of the pixel
	 * @return the global Z of 
	 * the pixel
	 */
	public float getZ(){
		return this.z;
	}
	
	
}
