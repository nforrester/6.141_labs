package motion;

import orc.Orc;
import orc.QuadratureEncoder;

import org.ros.message.rss_msgs.EncoderMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class EncoderPublisher implements Runnable {

	Node node;
	Orc orc;
	QuadratureEncoder left;
	QuadratureEncoder right;
	EncoderMsg msg;
	Publisher<EncoderMsg> pub;
	
	public EncoderPublisher(Node node, Orc orc) {
		this.node = node;
		this.orc = orc;
		left = new QuadratureEncoder(orc, 0, false);
		right = new QuadratureEncoder(orc, 1, true);
		pub = node.newPublisher("rss/Encoder", "rss_msgs/EncoderMsg");
	}
	
	@Override
	public void run() {
		msg = new EncoderMsg();
		while(true){
			msg.left = left.getPosition();
			msg.right = right.getPosition();
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
