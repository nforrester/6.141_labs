

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.AnalogStatusMsg;
import org.ros.message.sensor_msgs.PointCloud2;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;







public class KinectClean implements NodeMain{
	private ArrayBlockingQueue<PointCloud2> imageData = new ArrayBlockingQueue<PointCloud2>(1);
	private Publisher<org.ros.message.sensor_msgs.Image> vidPub;
	private Publisher<org.ros.message.rss_msgs.AnalogStatusMsg> interfacePub;
	public static int frame = 0;
	public static double frameNumber = 0;
	
	private Subscriber<PointCloud2> ptSub;
	private VisionTools visionTools;
	



	@Override
	public void onStart(Node node) {
		ptSub = node.newSubscriber("/camera/rgb/points","sensor_msgs/PointCloud2");
		ptSub.addMessageListener(new PointCloudListener());
		vidPub = node.newPublisher("/rss/kinectVideo", "sensor_msgs/Image");
		interfacePub = node.newPublisher("/rss/blockData","rss_msgs/AnalogStatusMsg");
		visionTools = new VisionTools();
		startTakingThread();
	}





	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
		if(node != null){
			node.shutdown();
		} 
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/kinectData");
	}


	class PointCloudListener implements MessageListener<PointCloud2>{
		@Override
		public void onNewMessage(PointCloud2 pc) {

			synchronized (imageData){
				imageData.offer(pc);
			}


		}
	}


	/**
	 * Makes a Kinect Image out
	 * of a Pointcloud2 message.
	 * 
	 * @param pc
	 * 			PointCloud2 message
	 * @return the Kinect Image
	 * 			
	 * @throws Exception if Point 
	 * 			cloud message contains
	 * 			invalid data.
	 */
	public KinectImage makeKinectImage(PointCloud2 pc) throws Exception{

		KinectImage image = new KinectImage((int)pc.width,(int)pc.height);

		for (int i = 0; i < pc.width * pc.height; i++) {

			int startByte = (int) (i * pc.point_step);
			int col = i / (int) pc.width;
			int row = i % (int) pc.width;

			float x = getFloat(pc.data, startByte);
			float y = getFloat(pc.data, startByte + 4);
			float z = getFloat(pc.data, startByte + 8);
			int blue = (int)(pc.data[startByte + 16] & 0xFF);
			int green = (int)(pc.data[startByte + 17] & 0xFF);
			int red = (int)(pc.data[startByte + 18] & 0xFF);

			KinectPixel storePixel = new KinectPixel(row,col,x,y,z,red,green,blue);
			image.setPixel(storePixel, row, col);


		}

		return image;
	}

/**
 * Makes the Image that
 * that can be published to
 * the screen for debugging
 * and analysis.
 * 
 * @param kinectImage Ki 
 * @return Image
 */
	public static Image makePublishImage(KinectImage ki){
		
		List<Image.Pixel> pixelList = new ArrayList<Image.Pixel>();
		
		for(int y = 0; y<ki.getHeight(); y++){
			for(int x = 0; x<ki.getWidth(); x++){
				KinectPixel currentPixel = ki.getPixel(x, y);
				pixelList.add(new Image.Pixel(currentPixel.getRed(),
						currentPixel.getGreen(),currentPixel.getBlue()));
			}
		}
		
		return new Image(pixelList,ki.getWidth(),ki.getHeight());
	}
	
	
	/**
	 * Gets a float value from the 
	 * byte array given by the kinect
	 * given the start index to read.
	 * 
	 * @param data
	 * 			the byte array 
	 * 			given by the kinect 
	 * @param startIdx
	 * 			the start index 
	 * 			from which 4 bytes have to
	 * 			be read in little endian
	 * 			format
	 * @return the float representation of
	 * 			4 bytes
	 */
	private float getFloat(byte[] data, int startIdx) {
		int fullInt = (data[startIdx + 0] & 0xFF)
				| ((data[startIdx + 1] & 0xFF) << 8)
				| ((data[startIdx + 2] & 0xFF) << 16)
				| ((data[startIdx + 3] & 0xFF) << 24);
		return Float.intBitsToFloat(fullInt);
	}



	/**
	 * Starts the message
	 * taking thread that takes 
	 * the message from the kinect
	 * and makes use of the messages.
	 */
	public void startTakingThread(){



		Thread t = new Thread(new Runnable(){
			@Override
			public void run() {

				while(true){
					frameNumber++;

					KinectImage newImage = null;

					try {
						PointCloud2 data = imageData.take();
						newImage = makeKinectImage(data);
					}catch(Exception e){
						System.err.println(e.toString());
					}
					KinectImage processedImage = newImage.copyImage();

					
					/////////////////////////////////////////////////////////////

					double[][] data = visionTools.blobPresent(newImage, processedImage);
					
					/////////////////////////////////////////////////////////////

					
					
					////////////////////// publish Image ///////////////////////
					
					Image transformedImage = makePublishImage(processedImage);
					
					org.ros.message.sensor_msgs.Image pubImage =	new org.ros.message.sensor_msgs.Image();
					pubImage.width = transformedImage.getWidth();
					pubImage.height = transformedImage.getHeight();
					pubImage.encoding = "rgb8";
					pubImage.is_bigendian = 0;
					pubImage.step = transformedImage.getWidth()*3;
					pubImage.data = transformedImage.toArray();
					vidPub.publish(pubImage);
					
					AnalogStatusMsg publishMessage = new AnalogStatusMsg();
					
					if(data[0][0] > 100){
						publishMessage.values = new double[]{frameNumber,0,data[0][3],data[0][4],data[0][5],0,0,0};
						interfacePub.publish(publishMessage);
					}
					if(data[0][1] > 100){
						publishMessage.values = new double[]{frameNumber,1,data[1][3],data[1][4],data[1][5],0,0,0};
						interfacePub.publish(publishMessage);
					}
					if(data[0][2] > 100){
						publishMessage.values = new double[]{frameNumber,2,data[2][3],data[2][4],data[2][5],0,0,0};
						interfacePub.publish(publishMessage);
					}
					if(data[0][3] > 100){
						publishMessage.values = new double[]{frameNumber,3,data[3][3],data[3][4],data[3][5],0,0,0};
						interfacePub.publish(publishMessage);
					}
					
				}


			}
		});

		t.start();



	}
	

}

