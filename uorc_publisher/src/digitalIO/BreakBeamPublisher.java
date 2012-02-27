package digitalIO;

import orc.DigitalInput;
import orc.Orc;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.rss_msgs.*;

public class BreakBeamPublisher implements Runnable {
	
	Node node;
	Orc orc;
	DigitalInput input;
	BreakBeamMsg msg;
	Publisher<BreakBeamMsg> pub;

	public BreakBeamPublisher(Node node, Orc orc) {
		// TODO Auto-generated constructor stub
		this.node = node;
		this.orc = orc;
		input = new DigitalInput(orc, 7, false, false);
		pub = node.newPublisher("rss/BreakBeam", "rss_msgs/BreakBeamMsg");
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		msg = new BreakBeamMsg();
		while (true){
			msg.beamBroken = input.getValue();
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

}
