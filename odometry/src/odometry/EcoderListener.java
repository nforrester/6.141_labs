package odometry;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.EncoderMsg;

public class EcoderListener implements MessageListener<EncoderMsg> {

	private Odometry parent;
	
	public EcoderListener(Odometry odometry) {
		parent = odometry;
	}

	@Override
	public void onNewMessage(EncoderMsg msg) {
		int[] ticks = new int[2];
		ticks[0] = (int) msg.left;
		ticks[1] = (int) msg.right;
		parent.update(ticks);
	}

}
