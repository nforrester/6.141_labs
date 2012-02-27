package odometry;

import odometry.EcoderListener;

import org.ros.message.rss_msgs.EncoderMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;

public class Odometry implements NodeMain {

	private int[] prev_ticks; // (left wheel, right wheel, make int[]
	private static final double WHEEL_RADIUS_IN_M = 0.0625;
	private static final double ENCODER_RESOLUTION = 2000;
	private static final double GEAR_RATIO = 65.5;
	private static final double TICKS_PER_REVOLUTION=ENCODER_RESOLUTION * GEAR_RATIO;
	private static final double WHEEL_METERS_PER_TICK = WHEEL_RADIUS_IN_M * Math.PI * 2/(TICKS_PER_REVOLUTION);
	private static final double WHEELBASE = .428;
	/**
	 * [x][y][theta]
	 */
	private double[] pose = {0, 0, 0};
	
	private OdometryMsg msg = new OdometryMsg();
	private Publisher<OdometryMsg> pub;
	private Subscriber<EncoderMsg> sub;
	
	public void update(int[] new_ticks) {
		if (prev_ticks == null && (new_ticks[0]==0 && new_ticks[1]==0)) {
			prev_ticks = new_ticks;
		}
		else if (prev_ticks == null) {
			prev_ticks = new int[2];
		}
		int[] ticks = new int[2];
		for (int i = 0; i < 2; i ++){
			ticks[i] = new_ticks[i] - prev_ticks[i];
		}
		if (ticks[0] == 0 && ticks[1] == 0) {
			pub.publish(msg);
			return; // we haven't moved at all
		}
		
		double s_left = (ticks[0])*WHEEL_METERS_PER_TICK;
		double s_right = (ticks[1])*WHEEL_METERS_PER_TICK;
		double theta = (s_left - s_right)/WHEELBASE;
		msg.theta += -theta;
		if(msg.theta < 0){
			msg.theta += 2* Math.PI;
		} else if (msg.theta > 2*Math.PI){
			msg.theta -=2*Math.PI;
		}
		msg.x += (s_left+s_right)*Math.cos(msg.theta)/2.0;
		msg.y += (s_left+s_right)*Math.sin(msg.theta)/2.0;
		prev_ticks[0] = new_ticks[0];
		prev_ticks[1] = new_ticks[1];
		pub.publish(msg);
	}

	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onStart(Node node) {
		pub = node.newPublisher("/rss/odometry", "rss_msgs/OdometryMsg");
		//sub = node.newSubscriber("/rss/Encoder", "rss_msgs/EncoderMsg", new EcoderListener(this));
		sub = node.newSubscriber("/rss/Encoder", "rss_msgs/EncoderMsg");
		sub.addMessageListener(new EcoderListener(this));
	}


    @Override
	public void onShutdownComplete(Node node) {
    }

    @Override
	public GraphName getDefaultNodeName() {
	return new GraphName("rss/odometry");
    }
}
