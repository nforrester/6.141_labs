package Motor;

import MotorControlSolution.RobotBase;
import MotorControlSolution.RobotVelocityControllerBalanced;
import MotorControlSolution.RobotVelocityController;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

/**
 * <p>Entry point for motor control lab.</p>
 **/
public class MotorControl implements NodeMain {

	@Override
	public void onShutdown(Node node){
		if (node != null){
			node.shutdown();
		}
	}

    @Override
	public void onShutdownComplete(Node node) {
    }
    
    @Override
	public GraphName getDefaultNodeName() {
	return new GraphName("rss/motorcontrol");
    }

    public Subscriber<MotionMsg> subscriber;

	/**
	 * <p>Entry point for the Motor Control lab.</p>
	 *
	 * @param args command line arguments
	 **/
	@Override
	public void onStart(Node node) {
		try{

			System.out.println("in main");

			RobotBase robot = new RobotBase();

			System.out.println("robot base made");

			RobotVelocityController robotVelocityController = null;

			//first task: direct pwm commanding

			// robotVelocityController =
			//  new RobotVelocityController(new WheelVelocityControllerPWM(),
			//                               new WheelVelocityControllerPWM());


			//second task: feed-forward control

			//    robotVelocityController =
			//      new RobotVelocityController(new WheelVelocityControllerFF(),
			//                                  new WheelVelocityControllerFF());


			//third task: integral feedback control

			//    robotVelocityController =
			//      new RobotVelocityController(new WheelVelocityControllerI(),
			//                                  new WheelVelocityControllerI());


			//final task: balanced velocity control

			robotVelocityController = new RobotVelocityControllerBalanced();
			System.out.println("robot velocity controller created");
			robot.setRobotVelocityController(robotVelocityController);

			robot.enableMotors(true);

			subscriber = node.newSubscriber("command/Motors", "rss_msgs/MotionMsg");
			MotorListener ml = new MotorListener(robotVelocityController);
			ml.log_node = node;
			subscriber.addMessageListener(ml);
			System.out.println("new Subscriber");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
