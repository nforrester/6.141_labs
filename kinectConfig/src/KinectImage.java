
public class KinectImage {

	
	private final int width;
	private final int height;
	private KinectPixel[][] pixels;
	
	/**
	 * Instansiates a new kinect 
	 * Image object.The pixels 
	 * of the image are not 
	 * instansiated.They need to be 
	 * instansiated seperately.
	 * 
	 * @param width
	 * 			the width of the
	 * 			image
	 * @param height
	 * 			the height of the 
	 * 			image
	 */
	public KinectImage(int width,int height){
		this.width = width;
		this.height =height;
		this.pixels = new KinectPixel[width][height];
	}
	
	/**
	 * Sets a particular pixel
	 * in the Kinect message.
	 * 
	 * @param pixel
	 * 			the kinectPixel
	 * 			object 
	 * @param row
	 * 			the row Pixel
	 * 			belongs to
	 * @param col
	 * 			the column pixel 
	 * 			belongs to. 
	 */
	public void setPixel(KinectPixel pixel,int row,int col){
		this.pixels[row][col] = pixel;
	}
	
	/**
	 * Gets the kinect pixel 
	 * in a particular position
	 * in the Image.
	 * 
	 * @param row
	 * 			the desired
	 * 			pixel row 
	 * @param col
	 * 			the desired 
	 * 			pixel col
	 * @return 
	 * 		The kinect pixel 
	 * 		object in row,col
	 */
	public KinectPixel getPixel(int row,int col){
		return this.pixels[row][col];
	}
	
	/**
	 * Gets the width of the Image
	 * @return the width
	 */
	public int getWidth(){
		return width;
	}
	
	/**
	 * Gets the height of the Image
	 * @return the height
	 */
	public int getHeight(){
		return height;
	}
	
	/**
	 * copies the image
	 * @param k
	 * @return
	 */
	public KinectImage copyImage(){
		KinectImage newImage = new KinectImage(this.getWidth(),this.getHeight());
		for (int i=0;i<this.getWidth();i++){
			for(int j=0;j<this.getHeight();j++){
				newImage.setPixel(this.getPixel(i, j).copyPixel(), i, j);
			}
		}
		return newImage;
	}
	
}
