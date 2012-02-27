import motion.EncoderPublisher;
import orc.Orc;

import org.ros.node.Node;
import org.ros.node.NodeMain;

import sonar.SonarPublisher;
import analogIO.AnalogIOPublisher;
import digitalIO.BreakBeamPublisher;
import digitalIO.BumperPublisher;
import digitalIO.DigitalIOPublisher;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

public class Publisher implements NodeMain {
	
	Orc orc;
	
	Thread digitalThread;
	
	Thread breakBeamThread;
	Thread bumpThread;
	
	Thread analogThread;
	
	Thread encoderThread;
	
	Thread frontSonarThread;
	Thread backSonarThread;

	@Override
	public void onStart(Node node) {
		// TODO Auto-generated method stub
		orc = Orc.makeOrc();
		
		DigitalIOPublisher digitalPub = new DigitalIOPublisher(node, orc);
		digitalThread = new Thread(digitalPub);
		digitalThread.start();
		
		BreakBeamPublisher breakBeamPub = new BreakBeamPublisher(node, orc);
		breakBeamThread = new Thread(breakBeamPub);
		breakBeamThread.start();
		
		BumperPublisher bumpPub = new BumperPublisher(node, orc);
		bumpThread = new Thread(bumpPub);
		bumpThread.start();
		
		AnalogIOPublisher analogPub = new AnalogIOPublisher(node, orc);
		analogThread = new Thread(analogPub);
		analogThread.start();
		
		EncoderPublisher encoderPub = new EncoderPublisher(node, orc);
		encoderThread = new Thread(encoderPub);
		encoderThread.start();
		
		SonarPublisher frontPub = new SonarPublisher(node, orc, true);
		SonarPublisher backPub = new SonarPublisher(node, orc, false);
		
		frontSonarThread = new Thread(frontPub);
		backSonarThread = new Thread(backPub);
		
		frontSonarThread.start();
		backSonarThread.start();
	}

	@SuppressWarnings("deprecation")
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub
		node.shutdown();
		digitalThread.stop();
		breakBeamThread.stop();
		bumpThread.stop();
		analogThread.stop();
		encoderThread.stop();
		frontSonarThread.stop();
		backSonarThread.stop();
	}

    @Override
	public void onShutdownComplete(Node node) {
    }

    @Override
	public GraphName getDefaultNodeName() {
	return new GraphName("rss/uorc_publisher");
    }

}
