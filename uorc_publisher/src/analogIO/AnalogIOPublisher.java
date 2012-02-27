package analogIO;

import orc.AnalogInput;
import orc.Orc;

import org.ros.message.rss_msgs.AnalogStatusMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class AnalogIOPublisher implements Runnable {

	Node node;
	Orc orc;
	AnalogInput[] inputs = new AnalogInput[8];
	AnalogStatusMsg msg;
	Publisher<AnalogStatusMsg> pub;
	
	public AnalogIOPublisher(Node node, Orc orc){
		this.node = node;
		this.orc = orc;
		for (int i = 0; i < 8; i++){
			inputs[i] = new AnalogInput(orc, i);
		}
		pub = node.newPublisher("rss/AnalogIO", "rss_msgs/AnalogStatusMsg");
	}
	
	@Override
	public void run() {
		msg = new AnalogStatusMsg();
		while(true){
			for (int i = 0; i < 8; i ++){
				msg.values[i] = inputs[i].getVoltage();
			}
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
