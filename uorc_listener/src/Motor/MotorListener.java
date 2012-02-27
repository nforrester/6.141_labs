package Motor;

import org.ros.message.MessageListener;

import MotorControlSolution.RobotVelocityController;
import org.ros.message.rss_msgs.MotionMsg;


public class MotorListener implements MessageListener<MotionMsg> {

    private RobotVelocityController controller;
	
    public MotorListener(RobotVelocityController rvc){
	controller = rvc;
    }
    
    @Override
	public void onNewMessage(MotionMsg msg){
	
	//System.out.println("got velocity command: " + msg.translationalVelocity + ", " + msg.rotationalVelocity);
	
	double left = msg.translationalVelocity;
	double right = msg.translationalVelocity;
	
	left -= msg.rotationalVelocity;
	right += msg.rotationalVelocity;
	
	left *= 3.5;
	right *= 3.5;
	controller.setDesiredAngularVelocity(left, right);
	
    }   
}