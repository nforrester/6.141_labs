package sonar;

import orc.Orc;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.std_msgs.Float64;

import orc.SRF02;

public class SonarPublisher implements Runnable{

	Node node;
	Orc orc;
	SRF02 sonar;
	boolean isFront;
	Publisher<SonarMsg> pub;
	final int frontAddr = 0x70;
	final int backAddr = 0x72;
	
	public static void main(String[] args){
		Orc orc = Orc.makeOrc();
		SRF02 one = new SRF02(orc);
		SRF02 two = new SRF02(orc, 0x72);
		while(true){
			System.out.println("distance for one:\t" + one.measure());
			System.out.println("distance for two:\t" + two.measure());
		}
	}
	public SonarPublisher(Node n, Orc o, boolean isFront){
		this.node = n;
		this.orc = o;
		this.isFront = isFront;
		if (isFront){
			sonar = new SRF02(orc, frontAddr);
			pub = node.newPublisher("rss/Sonars/Front", "rss_msgs/SonarMsg");
			System.out.println("created front sonar publisher");
		} else {
			sonar = new SRF02(orc, backAddr);
			pub = node.newPublisher("rss/Sonars/Back", "rss_msgs/SonarMsg");
			System.out.println("created back sonar publisher");
		}
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		SonarMsg msg = new SonarMsg();
		while(true){
			double s = sonar.measure();
			if (s!=0){
				msg.range = s;
			}
			msg.isFront = isFront;
			pub.publish(msg);
		}
	}

}
