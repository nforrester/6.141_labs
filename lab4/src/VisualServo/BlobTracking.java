package VisualServo;

import org.ros.node.Node;

import VisualServo.Image.Pixel;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
	public Node log_node;

	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;
	public int numChannels = 3;

	private double RED_HUE   = 0.0 / 3.0;
	private double GREEN_HUE = 1.0 / 3.0;
	private double BLUE_HUE  = 2.0 / 3.0;
	private double hues[] = {RED_HUE, GREEN_HUE, BLUE_HUE};

	private int RED   = 0;
	private int GREEN = 1;
	private int BLUE  = 2;

	private int preference = RED;

	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()

	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param image width
	 * @param image height
	 */
	public BlobTracking(int width, int height) {

		this.width = width;
		this.height = height;

	}	
	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			double fps = (double) stepCounter * 1000.0
			/ (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}
	
	
	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 *This method turns the matching pixels to white and prints out an 
	 *estimated center of the ball. 
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public void apply(Image src, Image dest) {
		
		long redPositionSumX = 0;
		long redPositionSumY = 0;
		int numberOfPixels = 0;
		
		stepTiming();  // monitors the frame rate
		//log_node.getLog().info( src.getPixel(100, 100).getBlue());
		// Begin Student Code
		log_node.getLog().info("adjbgvfqaj");
		for(int i=0;i<src.getWidth();i++) {
		    for(int j=0;j<src.getHeight();j++) {
		        Pixel testPixel = src.getPixel(i, j);
		        if(testPixel.getRed() > 220 && testPixel.getBlue() < 20 && testPixel.getGreen() < 20) {
		            System.out.println("Pixel at i = " + i + " j = " + j);
		        }
		    }
		}

		Histogram.getHistogram(src, dest, false);
		
		for(int i=0;i<src.getWidth();i++){
			for(int j=0;j<src.getHeight();j++){
				
		
				Pixel testPixel = src.getPixel(i, j);
				
				if(isHue(testPixel, hues[RED])){
					Pixel redPixel = new Pixel(255,255,255);
					dest.setPixel(i, j, redPixel); //replaces desired pixels with white pixels.
					numberOfPixels++;
					redPositionSumX += i;
					redPositionSumY += j;
				}
			}
		}
		
		if(numberOfPixels !=0){
			redPositionSumX /= (long)numberOfPixels;
			redPositionSumY /= (long)numberOfPixels;
		}
		
		
		
		if(redPositionSumX > 0 && redPositionSumY > 0){
			log_node.getLog().info("Ball Found Estimated center at X -> " + redPositionSumX + "Y -> " +redPositionSumY);
		}
		// End Student Code
	}
	
	/*
	 * Tests if a pixel is a particular hue,
         * with reasonable saturation and brightness
         * to make such a determination
	 */
	public boolean isHue(Pixel p, double hue) {
		return isHue(p, hue, 0.05);
	}
	public boolean isHue(Pixel p, double hue, double tolerance) {
		double pixHue = p.getHue();

		if (p.getBrightness() < 0.35 || p.getSaturation() < 0.5) {
			return false;
		}

		pixHue -= hue;

		while (pixHue > 0.5) {
			pixHue -= 1;
		}
		while (pixHue < -0.5) {
			pixHue += 1;
		}

		return Math.abs(pixHue) < tolerance;
	}

	/*
	 * Classifies each pixel in the image as target pixel or not and converts 
	 * classified targetPixels into saturated colors, eg. Pixel(255,0,0) for a 
	 * red ball and non- blob pixels are converted into gray-scale.
	 */
	public void classify(Image src, Image dest) {
		
		stepTiming(); 
		
		
		for(int i=0;i<src.getWidth();i++){
			for(int j=0;j<src.getHeight();j++){
				
				Pixel testPixel = src.getPixel(i, j);
				
				if(isHue(testPixel, hues[preference])){
					Pixel saturatedPixel;

					if (preference == RED) {
						saturatedPixel = new Pixel(255,   0,   0);
					} else if (preference == GREEN) {
						saturatedPixel = new Pixel(  0, 255,   0);
					} else {
						saturatedPixel = new Pixel(  0,   0, 255);
					}

					dest.setPixel(i, j, saturatedPixel); 
				}
				else{
					int redValue = testPixel.getRed();
					int blueValue = testPixel.getBlue();
					int greenValue = testPixel.getGreen();
					int grayScale = (redValue + blueValue + greenValue)/3;
					
					Pixel grayPixel = new Pixel(grayScale,grayScale,grayScale);
					dest.setPixel(i, j, grayPixel); 
				}	
			}
		}
		
		
	}
	
	/*
	 * This  method counts the number of target pixels in the frame, calculates the centroid and
	 * draws a cross at the centroid of the ball.It also does the classification of the image using 
	 * classify(Image,Image) in BlobTracking.
	 * 
	 * @param
	 * src - Source image from the camera
	 * dest - The Destination image that is going to 
	 * 		  the display for being displayed.
	 * 
	 * @return
	 * returns an int[]  consisting of Area,X,Y of the blob found on the screen.  
	 * 
	 */
	public int[] blobPresent(Image src, Image dest){
		
		classify(src,dest);
		
		stepTiming(); 
		
		int[] returnArray = new int[3];
		int matchingPixels = 0;
		long matchingPixelsXCount = 0;
		long matchingPixelsYCount = 0;
		
		for(int i=0;i<dest.getWidth();i++){
			for(int j=0;j<dest.getHeight();j++){
				Pixel nextPixel = dest.getPixel(i, j);
				
				boolean condition = (preference == RED   && nextPixel.getRed() == 255 && nextPixel.getGreen() ==   0 && nextPixel.getBlue() ==   0) ||
				                    (preference == GREEN && nextPixel.getRed() ==   0 && nextPixel.getGreen() == 255 && nextPixel.getBlue() ==   0) ||
				                    (preference == BLUE  && nextPixel.getRed() ==   0 && nextPixel.getGreen() ==   0 && nextPixel.getBlue() == 255);
								
				if (condition){
					matchingPixels++;
					matchingPixelsXCount += i;
					matchingPixelsYCount += j;
				}
				
			}
		}
		
		if(matchingPixels >50){
			matchingPixelsXCount /= (long)matchingPixels;
			matchingPixelsYCount /= (long)matchingPixels;
		}
		else{
			matchingPixelsXCount = 0;
			matchingPixelsYCount = 0;
		}
		
		returnArray[0] = (int) matchingPixels; //area
		returnArray[1] = (int)matchingPixelsXCount - (int)(dest.getWidth()/2); // Averaged X with (0,0) as center of image
		returnArray[2] = (int)matchingPixelsYCount - (int)(dest.getHeight()/2); // Averaged Y with (0,0) as center of image
		
		if(matchingPixelsXCount > 0 && matchingPixelsYCount > 0){
			for (int i = (int)matchingPixelsXCount - 8; i< (int)matchingPixelsXCount + 9;i++){
				Pixel whitePixel = new Pixel(255,255,255);
				if(i < dest.getWidth() && i > 0){
					dest.setPixel(i,(int)matchingPixelsYCount , whitePixel);
				}
			}
			for (int i = (int)matchingPixelsYCount - 8; i< (int)matchingPixelsYCount + 9;i++){
				Pixel whitePixel = new Pixel(255,255,255);
				if(i < dest.getHeight() && i > 0){
					dest.setPixel((int)matchingPixelsXCount ,i, whitePixel);
				}
			}
			
		}
		
		if (true) {
			Histogram.getHistogram(src, dest, false);
		}

		return returnArray;
	}
}
