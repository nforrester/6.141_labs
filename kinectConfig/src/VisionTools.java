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





	public ArrayList<double[]> multipleBlobsPresent(KinectImage src, KinectImage dest){

		// width -> 640
		// height -> 480

		//classify blobs
		this.classify(src,dest);

		int[][][] matchingPixelsCounts = new int[4][16][12];
		long[][][] matchingPixelsXCount = new long[4][16][12];
		long[][][] matchingPixelsYCount = new long[4][16][12];
		boolean[][][] conditionSatisfied = new boolean[4][16][12];

		ArrayList<double[]> returnArray = new ArrayList<double[]>();


		for(int i=0;i<dest.getWidth();i++){
			for(int j=0;j<dest.getHeight();j++){

				KinectPixel testPixel = src.getPixel(i, j);

				if(isHueSatisfied(testPixel, blockHues[RED])){
					matchingPixelsXCount[RED][(int)(i/40)][(int)(j/40)] += i;
					matchingPixelsYCount[RED][(int)(i/40)][(int)(j/40)] += j;
					matchingPixelsCounts[RED][(int)(i/40)][(int)(j/40)]++;
				}
				else if(isHueSatisfied(testPixel, blockHues[GREEN])){
					matchingPixelsXCount[GREEN][(int)(i/40)][(int)(j/40)] += i;
					matchingPixelsYCount[GREEN][(int)(i/40)][(int)(j/40)] += j;
					matchingPixelsCounts[GREEN][(int)(i/40)][(int)(j/40)]++;
				}
				else if(isHueSatisfied(testPixel, blockHues[BLUE])){
					matchingPixelsXCount[BLUE][(int)(i/40)][(int)(j/40)] += i;
					matchingPixelsYCount[BLUE][(int)(i/40)][(int)(j/40)] += j;
					matchingPixelsCounts[BLUE][(int)(i/40)][(int)(j/40)]++;
				}
				else if(isHueSatisfied(testPixel, blockHues[YELLOW])){
					matchingPixelsXCount[YELLOW][(int)(i/40)][(int)(j/40)] += i;
					matchingPixelsYCount[YELLOW][(int)(i/40)][(int)(j/40)] += j;
					matchingPixelsCounts[YELLOW][(int)(i/40)][(int)(j/40)]++;
				}

			}
		}


		for(int i=0;i<4;i++){
			for(int j= 0;j<16;j++){
				for(int k=0;k<12;k++){

					if(matchingPixelsCounts[i][j][k] > 100){
						matchingPixelsXCount[i][j][k] /= (long)matchingPixelsCounts[i][j][k];
						matchingPixelsYCount[i][j][k] /= (long)matchingPixelsCounts[i][j][k];
						conditionSatisfied[i][j][k] = true;
					}
					else{
						matchingPixelsXCount[i][j][k] = 0;
						matchingPixelsYCount[i][j][k] = 0;
					}

				}
			}
		}





		for(int i=0;i<4;i++){
			for(int j= 0;j<16;j++){
				for(int k=0;k<12;k++){
					boolean one = false;
					boolean two = false;
					boolean three = false;
					if(j+1 < 16){
						if((conditionSatisfied[i][j][k] && conditionSatisfied[i][j+1][k])){
							one = true;
							matchingPixelsCounts[i][j+1][k] = matchingPixelsCounts[i][j+1][k] + 
									matchingPixelsCounts[i][j][k];

							matchingPixelsXCount[i][j+1][k] = (matchingPixelsXCount[i][j+1][k] + 
									matchingPixelsXCount[i][j][k])/2;

							matchingPixelsYCount[i][j+1][k] = (matchingPixelsYCount[i][j+1][k] + 
									matchingPixelsYCount[i][j][k])/2;
						}
					}

					if(k +1 <12){
						if((conditionSatisfied[i][j][k] && conditionSatisfied[i][j][k+1])){
							two = true;
							matchingPixelsCounts[i][j][k+1] = matchingPixelsCounts[i][j][k + 1] + 
									matchingPixelsCounts[i][j][k];

							matchingPixelsXCount[i][j][k+ 1] = (matchingPixelsXCount[i][j][k+1] + 
									matchingPixelsXCount[i][j][k])/2;

							matchingPixelsYCount[i][j][k+1] = (matchingPixelsYCount[i][j][k+1] + 
									matchingPixelsYCount[i][j][k])/2;
						}
					}

					if(j+1 <16 && k+1 < 12){
						if((conditionSatisfied[i][j][k] && conditionSatisfied[i][j+1][k+1])){
							three = true;
							matchingPixelsCounts[i][j+1][k+1] = matchingPixelsCounts[i][j+1][k + 1] + 
									matchingPixelsCounts[i][j][k];

							matchingPixelsXCount[i][j +1][k+ 1] = (matchingPixelsXCount[i][j+1][k+1] + 
									matchingPixelsXCount[i][j][k])/2;

							matchingPixelsYCount[i][j +1][k+1] = (matchingPixelsYCount[i][j+1][k+1] + 
									matchingPixelsYCount[i][j][k])/2;
						}
					}

					if(one || two || three){
						conditionSatisfied[i][j][k] = false;
					}
				}
			}
		}


		
		for(int i=0;i<4;i++){
			for(int j= 0;j<16;j++){
				for(int k=0;k<12;k++){
					if(conditionSatisfied[i][j][k]){
						
						double[] depthData = getDepthData((int)matchingPixelsXCount[i][j][k],
															(int)matchingPixelsYCount[i][j][k],src);
						double[] data = {i,matchingPixelsCounts[i][j][k],
										(int)matchingPixelsXCount[i][j][k] - (int)(dest.getWidth())/2,
										(int)matchingPixelsYCount[i][j][k] - (int)(dest.getWidth())/2,
										(depthData[0] > 5) ? -100 : depthData[0],
										(depthData[1] > 5) ? -100 : depthData[1],
										(depthData[2] > 5) ? -100 : depthData[2]};
						returnArray.add(data);
						
					}
				}
			}
		}
		
		//print data
		for(double[] a : returnArray){
			System.err.println("Color -> " + a[0] + "X -> " + a[4] + "Y -> " + a[5]+ "Z -> " + a[6]);
			
		}


		return returnArray;
	}








	public double[][] blobPresent(KinectImage src, KinectImage dest){

		this.classify(src,dest);

		double[][] returnArray = new double[4][6];
		int[] matchingPixels = new int[4];
		long[] matchingPixelsXCount = new long[4];
		long[] matchingPixelsYCount = new long[4];

		System.err.println("width -> " + dest.getWidth() );
		System.err.println("height -> " + dest.getHeight() );


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
			if(matchingPixels[i] > 100){
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
			double[] depthData = getDepthData((int)matchingPixelsXCount[i],(int)matchingPixelsYCount[i],src);
			returnArray[i][3] = (depthData[0] > 5) ? -100 : depthData[0];
			returnArray[i][4] = (depthData[1] > 5) ? -100 : depthData[1];
			returnArray[i][5] = (depthData[2] > 5) ? -100 : depthData[2];


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


		/*System.err.println("Red Area -> " + returnArray[0][0] + "Red X -> " + returnArray[0][1]
					+ "Red Y -> " + returnArray[0][2]);
			System.err.println("green Area -> " + returnArray[1][0] + "green X -> " + returnArray[1][1]
					+ "green Y -> " + returnArray[1][2]);
			System.err.println("blue Area -> " + returnArray[2][0] + "blue X -> " + returnArray[2][1]
					+ "blue Y -> " + returnArray[2][2]);
			System.err.println("yellow Area -> " + returnArray[3][0] + "yellow X -> " + returnArray[3][1]
					+ "yellow Y -> " + returnArray[3][2]);
			System.err.println();*/

		System.err.println("Red X -> " + returnArray[0][3] + "Red Y -> " + returnArray[0][4]
				+ "Red Z -> " + returnArray[0][5]);
		System.err.println("green X -> " + returnArray[1][3] + "green Y -> " + returnArray[1][4]
				+ "green Z -> " + returnArray[1][5]);
		System.err.println("blue X -> " + returnArray[2][3] + "blue Y -> " + returnArray[2][4]
				+ "blue Z -> " + returnArray[2][5]);
		System.err.println("yellow X -> " + returnArray[3][3] + "yellow Y -> " + returnArray[3][4]
				+ "yellow Z -> " + returnArray[3][5]);
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


	private double[] getDepthData(int row,int col,KinectImage src) {
		KinectPixel testPixel = src.getPixel(row, col);
		if(!Float.isNaN(testPixel.getX()) && !Float.isNaN(testPixel.getY()) && !Float.isNaN(testPixel.getZ())){
			return new double[]{testPixel.getX(),testPixel.getY(),testPixel.getZ()};
		}
		else{

			if(isValid(row,col-1,src)){
				return getDepthData(row,col-1,src);
			}

		}
		return new double[]{-100,-100,-100};
	}



	private boolean isValid(int xVal,int yVal,KinectImage src) {
		return xVal >=0 && yVal >=0 && xVal < src.getWidth() && yVal < src.getHeight();
	}

	private ArrayList<int[]> getNeighborsOne(int x,int y) {
		ArrayList<int[]> neighborList = new ArrayList<int[]>();
		neighborList.add(new int[]{x,y-1});
		return neighborList;
	}



}

