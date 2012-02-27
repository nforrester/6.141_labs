package digitalIO;

import orc.AnalogInput;
import orc.Orc;

import org.ros.message.rss_msgs.BumpMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class BumperPublisher implements Runnable {
	
	public static final String BUMP_MSG = "rss_msgs/BumpMsg";


	public static final String BUMP_CHANNEL = "rss/BumpSensors";
	
	
	private Node node;
	private Orc orc;
	private AnalogInput left;
	private AnalogInput right;
	private BumpMsg msg;
	private Publisher<BumpMsg> pub;
	private double epsilon = .01;
	
	public BumperPublisher(Node node, Orc orc){
		this.node = node;
		this.orc = orc;
		left = new AnalogInput(orc, 0);
		right = new AnalogInput(orc, 1);
		pub = node.newPublisher(BUMP_CHANNEL, BUMP_MSG);
	}
	
	@Override
	public void run() {
		msg = new BumpMsg();
		while(true){
			double leftVal = left.getVoltage();
			double rightVal = right.getVoltage();
			msg.left = (leftVal < epsilon );
			msg.right = (rightVal < epsilon);
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
