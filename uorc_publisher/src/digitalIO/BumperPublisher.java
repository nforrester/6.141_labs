package digitalIO;

import orc.AnalogInput;
import orc.DigitalInput;
import orc.Orc;

import org.ros.message.rss_msgs.BumpMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class BumperPublisher implements Runnable {
	
	public static final String BUMP_MSG = "rss_msgs/BumpMsg";


	public static final String BUMP_CHANNEL = "rss/BumpSensors";
	
	
	private Node node;
	private Orc orc;
	private DigitalInput left;
	private DigitalInput right;
	private BumpMsg msg;
	private Publisher<BumpMsg> pub;
	
	public BumperPublisher(Node node, Orc orc){
		this.node = node;
		this.orc = orc;
		left = new DigitalInput(orc, 0, true, true);
		right = new DigitalInput(orc, 1, true, true);
		pub = node.newPublisher(BUMP_CHANNEL, BUMP_MSG);
	}
	
	@Override
	public void run() {
		msg = new BumpMsg();
		while(true){
			msg.left = left.getValue();
			msg.right = right.getValue();
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
