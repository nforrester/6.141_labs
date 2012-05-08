import java.awt.geom.Point2D;

import java.awt.geom.Point2D.Double;
import java.util.ArrayList;




public class VisionTools {

	private static final double RED_BLOCK_HUE   = 0.99203705;
	private static final double GREEN_BLOCK_HUE = 0.27136752;
	private static final double BLUE_BLOCK_HUE  = 0.6666666;
	private static final double YELLOW_BLOCK_HUE = 0.16084425; 
	private static final double blockHues[] = {RED_BLOCK_HUE, GREEN_BLOCK_HUE, BLUE_BLOCK_HUE,YELLOW_BLOCK_HUE};

	private static final double GREEN_FIDUCIAL_HUE = 0.20290598;
	private static final double YELLOW_FIDUCIAL_HUE  = 0.16083703;
	private static final double RED_FIDUCIAL_HUE = 0.025; 
	private static final double BLUE_FIDUCIAL_HUE = 0.6;  // yet to be caliberated
	private static final double ORANGE_FIDUCIAL_HUE   = 0.079365075; // yet to be caliberated
	private static final double fiducialHues[] = {RED_FIDUCIAL_HUE, GREEN_FIDUCIAL_HUE, 
		BLUE_FIDUCIAL_HUE,YELLOW_FIDUCIAL_HUE,ORANGE_FIDUCIAL_HUE};


	private static final int RED   = 0;
	private static final int GREEN = 1;
	private static final int BLUE  = 2;
	private static final int YELLOW  = 3;
	private static final int ORANGE  = 4;





	public int[][] blobPresent(KinectImage src, KinectImage dest){

		this.classify(src,dest);

		int[][] returnArray = new int[4][3];
		int[] matchingPixels = new int[4];
		long[] matchingPixelsXCount = new long[4];
		long[] matchingPixelsYCount = new long[4];

		for(int i=0;i<dest.getWidth();i++){
			for(int j=0;j<dest.getHeight();j++){

				KinectPixel testPixel = src.getPixel(i, j);

				if(isHueSatisfied(testPixel, blockHues[RED])){
					matchingPixels[RED]++;
					matchingPixelsXCount[RED] += i;
					matchingPixelsYCount[RED] += j;
				}
				else if(isHueSatisfied(testPixel, blockHues[GREEN])){
					matchingPixels[GREEN]++;
					matchingPixelsXCount[GREEN] += i;
					matchingPixelsYCount[GREEN] += j;
				}
				else if(isHueSatisfied(testPixel, blockHues[BLUE])){
					matchingPixels[BLUE]++;
					matchingPixelsXCount[BLUE] += i;
					matchingPixelsYCount[BLUE] += j;
				}
				else if(isHueSatisfied(testPixel, blockHues[YELLOW])){
					matchingPixels[YELLOW]++;
					matchingPixelsXCount[YELLOW] += i;
					matchingPixelsYCount[YELLOW] += j;
				}

			}
		}

		for(int i=0;i<4;i++){
			if(matchingPixels[i] > 500){
				matchingPixelsXCount[i] /= (long)matchingPixels[i];
				matchingPixelsYCount[i] /= (long)matchingPixels[i];
			}
			else{
				matchingPixelsXCount[i] = 0;
				matchingPixelsYCount[i] = 0;
			}


			returnArray[i][0] = (int) matchingPixels[i]; //area
			returnArray[i][1] = (int)matchingPixelsXCount[i] - (int)(dest.getWidth()/2); // Averaged X with (0,0) as center of image
			returnArray[i][2] = (int)matchingPixelsYCount[i] - (int)(dest.getHeight()/2); // Averaged Y with (0,0) as center of image

			if(matchingPixelsXCount[i] > 0 && matchingPixelsYCount[i] > 0){
				for (int j = (int)matchingPixelsXCount[i] - 8; j< (int)matchingPixelsXCount[i] + 9;j++){
				
					if(j < dest.getWidth() && j > 0){
						dest.setPixel(dest.getPixel(j,(int)matchingPixelsYCount[i]).changeColor(255,   255,   255),
								j,(int)matchingPixelsYCount[i]);

					}
				}
				for (int j = (int)matchingPixelsYCount[i] - 8; j< (int)matchingPixelsYCount[i] + 9;j++){
					
					if(j < dest.getHeight() && j > 0){
						dest.setPixel(dest.getPixel((int)matchingPixelsYCount[i],j).changeColor(255,   255,   255),
								(int)matchingPixelsYCount[i],j);
					}
				}

			}

		}
		
	
			System.err.println("Red Area -> " + returnArray[0][0] + "Red X -> " + returnArray[0][1]
					+ "Red Y -> " + returnArray[0][2]);
			System.err.println("green Area -> " + returnArray[1][0] + "green X -> " + returnArray[1][1]
					+ "green Y -> " + returnArray[1][2]);
			System.err.println("blue Area -> " + returnArray[2][0] + "blue X -> " + returnArray[2][1]
					+ "blue Y -> " + returnArray[2][2]);
			System.err.println("yellow Area -> " + returnArray[3][0] + "yellow X -> " + returnArray[3][1]
					+ "yellow Y -> " + returnArray[3][2]);
			System.err.println();
		
		
		return returnArray;
	}






	/**
	 * Classifies the convex Hull of the pixels.
	 * @param the source image
	 * @param the destination image
	 */
	public void classifyHulls(KinectImage src, KinectImage dest) {
		ArrayList<Point2D.Double> redValues = new ArrayList<Point2D.Double>();
		ArrayList<Point2D.Double> greenValues = new ArrayList<Point2D.Double>();
		ArrayList<Point2D.Double> blueValues = new ArrayList<Point2D.Double>();
		ArrayList<Point2D.Double> yellowValues = new ArrayList<Point2D.Double>();
		ArrayList<Point2D.Double> orangeValues = new ArrayList<Point2D.Double>();



		for(int i=0;i<src.getWidth();i++){
			for(int j=0;j<src.getHeight();j++){


				KinectPixel testPixel = src.getPixel(i, j);

				if(isHueSatisfied(testPixel, blockHues[RED]) || isHueSatisfied(testPixel, fiducialHues[RED]) ){
					redValues.add(new Point2D.Double((double)i,(double)j));
				}
				else if(isHueSatisfied(testPixel, blockHues[GREEN])|| isHueSatisfied(testPixel, fiducialHues[GREEN])){
					greenValues.add(new Point2D.Double((double)i,(double)j));
				}
				else if(isHueSatisfied(testPixel, blockHues[BLUE])|| isHueSatisfied(testPixel, fiducialHues[BLUE])){
					blueValues.add(new Point2D.Double((double)i,(double)j));
				}
				else if(isHueSatisfied(testPixel, blockHues[YELLOW])|| isHueSatisfied(testPixel, fiducialHues[YELLOW])){
					yellowValues.add(new Point2D.Double((double)i,(double)j));
				}
				else if(isHueSatisfied(testPixel, fiducialHues[ORANGE])){
					orangeValues.add(new Point2D.Double((double)i,(double)j));
				}


				int redValue = testPixel.getRed();
				int blueValue = testPixel.getBlue();
				int greenValue = testPixel.getGreen();
				int grayScale = (redValue + blueValue + greenValue)/3;

				KinectPixel grayPixel = testPixel.changeColor(grayScale,   grayScale,   grayScale);
				dest.setPixel(grayPixel,i,j); 
			}
		}



		if(redValues.size() > 10) {
			ArrayList<Point2D.Double> redHull = ConvexHull.getConvexHull(redValues);
			for(Point2D.Double p : redHull){
				dest.setPixel(dest.getPixel((int)p.x,(int)p.y).changeColor(255,   0,   0),(int)p.x,(int)p.y);
			}
		}
		if(greenValues.size() > 10)  {
			greenValues = ConvexHull.getConvexHull(greenValues);
			for(Point2D.Double p : greenValues){
				dest.setPixel(dest.getPixel((int)p.x,(int)p.y).changeColor(255,   0,   0),(int)p.x,(int)p.y);
			}
		}

		if(blueValues.size() > 10) {
			blueValues = ConvexHull.getConvexHull(blueValues);

			for(Point2D.Double p : blueValues){
				dest.setPixel(dest.getPixel((int)p.x,(int)p.y).changeColor(255,   0,   0),(int)p.x,(int)p.y);
			}
		}

		if(yellowValues.size() > 10){
			yellowValues = ConvexHull.getConvexHull(yellowValues);
			for(Point2D.Double p : yellowValues){
				dest.setPixel(dest.getPixel((int)p.x,(int)p.y).changeColor(255,   0,   0),(int)p.x,(int)p.y);
			}
		}

		if(orangeValues.size() > 10){
			orangeValues = ConvexHull.getConvexHull(orangeValues);
			for(Point2D.Double p : orangeValues){
				dest.setPixel(dest.getPixel((int)p.x,(int)p.y).changeColor(255,   0,   0),(int)p.x,(int)p.y);
			}
		}

	}






	/**
	 * Classifies each pixel in the image as target pixel or not and converts 
	 * classified targetPixels into saturated colors, eg. Pixel(255,0,0) for a 
	 * red ball and non- blob pixels are converted into gray-scale.
	 * @param the source image
	 * @param the destination image
	 */
	public void classify(KinectImage src, KinectImage dest) {

		for(int i=0;i<src.getWidth();i++){
			for(int j=0;j<src.getHeight();j++){

				KinectPixel testPixel = src.getPixel(i, j);

				if(isHueSatisfied(testPixel, blockHues[RED]) || isHueSatisfied(testPixel, fiducialHues[RED]) ){
					KinectPixel saturatedPixel = testPixel.changeColor(255,   0,   0);
					dest.setPixel(saturatedPixel,i,j); 
				}
				else if(isHueSatisfied(testPixel, blockHues[GREEN])|| isHueSatisfied(testPixel, fiducialHues[GREEN])){
					KinectPixel saturatedPixel = testPixel.changeColor(0,   255,   0);
					dest.setPixel(saturatedPixel,i,j);
				}
				else if(isHueSatisfied(testPixel, blockHues[BLUE])|| isHueSatisfied(testPixel, fiducialHues[BLUE])){
					KinectPixel saturatedPixel = testPixel.changeColor(0,   0,   255);
					dest.setPixel(saturatedPixel,i,j);
				}
				else if(isHueSatisfied(testPixel, blockHues[YELLOW])|| isHueSatisfied(testPixel, fiducialHues[YELLOW])){
					KinectPixel saturatedPixel = testPixel.changeColor(255,   255,   0);
					dest.setPixel(saturatedPixel,i,j);
				}
				else if(isHueSatisfied(testPixel, fiducialHues[ORANGE])){
					KinectPixel saturatedPixel = testPixel.changeColor(255,   127,   80);
					dest.setPixel(saturatedPixel,i,j);
				}
				else{
					int redValue = testPixel.getRed();
					int blueValue = testPixel.getBlue();
					int greenValue = testPixel.getGreen();
					int grayScale = (redValue + blueValue + greenValue)/3;

					KinectPixel grayPixel = testPixel.changeColor(grayScale,   grayScale,   grayScale);
					dest.setPixel(grayPixel,i,j); 
				}

			}
		}
	}





	/**
	 * Tests if a pixel is a particular hue,
	 * with reasonable saturation and brightness
	 * to make such a determination
	 * @param Pixel to compare 
	 * @param hue to compare with
	 * @return true if satisfied, false otherwise
	 */
	public boolean isHueSatisfied(KinectPixel p, double hue) {
		return isHueSatisfied(p, hue, 0.02);
	}

	public boolean isHueSatisfied(KinectPixel p, double hue, double tolerance) {
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



}

