package Challenge.PhaseTwo;

import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Challenge.Waypoint;
import Grasping.Grasping;
import Grasping.GripperController;
import Grasping.ShoulderController;
import Grasping.WristController;
import Grasping.Grasping.ArmListener;

/**
 * <p>Deals with Contructing Blocks</p>
 *
 * @author dgonz
 *
 **/

public class PhaseTwoController implements NodeMain, Runnable{
	
	//State Stuff
	public static final int INIT=0;
	public static final int DEPOSIT_BLOCKS=1;
	public static final int PICK_UP=2;
	public static final int PLACE=3;
	public static final int DONE=4;
	public static final int DEBUG=5;
	public static final int LOCALIZE=6;
	public static final int GO_TO_NEXT=6;
	
	private boolean currentBreakBeamValue = false;
	
	private int state;
	private int towerHeight=5;
	private int iterator=1;
	
	//Measured Stuff
	public static final double BLOCK_SIZE=2.07*Manipulator.I2M;
	
	public final double[][] DATA={{600,500,0},
								  {725,800,-.08},
								  {900,1000,-.115},
								  {1010,1150,-.135},
								  {1050,1250,-.145}};
	
	//ROS stuff
	private Node node;
	private Publisher<org.ros.message.std_msgs.String> statePub;
	private Subscriber<org.ros.message.rss_msgs.BreakBeamMsg> breakBeamSub;
	private org.ros.message.std_msgs.String stateMsg;
	
	
	//Controller Stuff
	double x=0; //end effector x
	double y=0; //end effector y
	double theta=0;//end effector theta	
	double xStart=0;//Starting X after Localization
	
	Manipulator manipulator;
	PhaseTwoMotorController robotController;
	
	
	
	public void waitUp(int val){
		System.err.println("Waiting "+val+" microseconds.");
		try {
			Thread.sleep(val);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void goToY(double yNew){
		manipulator.goToY(yNew);
		y=yNew;
		
	}
	
	public double percentToY(double val){
		return (val/100)*(manipulator.Y_MAX-manipulator.Y_MIN)+manipulator.Y_MIN;
	}
	
	//Main FSM Handles
	private void step(){
		if(state==INIT){

			System.err.println("Init");
			changeState(DEPOSIT_BLOCKS);
			//changeState(DEBUG);
			//changeState(PICK_UP);
		
		}else if(state==DEPOSIT_BLOCKS){

			System.err.println("deposit");
		    //assumes that odometry has been reset at the beginning of phase 2		        
		    robotController.goToX(-2*15*Manipulator.I2M);
		    waitUp(1000);
		    while(robotController.getIsMoving()){
		    }

			System.err.println("Done Depositing!");
		    changeState(LOCALIZE);
		    
		}else if(state==LOCALIZE){
			System.err.println("localize");
			manipulator.goToPickUp2();
			waitUp(1000);
			while(!currentBreakBeamValue){//while(IR Sensor is unbroken)
				robotController.setTranslationalVelocity(.1);
			}
			
			robotController.setTranslationalVelocity(0);
			xStart=x;
			changeState(PICK_UP);
			
		}else if(state==PICK_UP){

			System.err.println("pick up");
			manipulator.goToPickUp2();
			waitUp(2000);
			manipulator.closeGripper();
			waitUp(2000);
			robotController.goToX(DATA[iterator][2]+xStart);
			System.err.println(robotController.getIsMoving());
			waitUp(2000);
		    while(robotController.getIsMoving()){
		    }
	        manipulator.servoOut2((int)DATA[iterator][0],(int)DATA[iterator][1],500);
		    waitUp(2000);
		    changeState(PLACE);
		    
		}else if(state==PLACE){
			
			System.err.println("place");
			robotController.goToX(towerHeight-iterator)*BLOCK_SIZE+DATA[iterator][2]+xStart);
			waitUp(2000);
		    while(robotController.getIsMoving()){
		    }
		    manipulator.openGripper();
		    waitUp(2000);
		    iterator++;
		    changeState(GO_TO_NEXT);
		}else if(state==GO_TO_NEXT){
			if (iterator==towerHeight){
				robotController.goToX(xStart);
				waitUp(2000);
			    while(robotController.getIsMoving()){
			    }
				changeState(DONE);
			}else{
				robotController.goToX((iterator-1)*BLOCK_SIZE+DATA[iterator][2]+xStart);
				waitUp(2000);
			    while(robotController.getIsMoving()){
			    }
			    manipulator.goToPickUp2();
			    waitUp(2000);
			    robotController.goToX((iterator-1)*BLOCK_SIZE+xStart);
			    waitUp(2000);
			    while(robotController.getIsMoving()){
			    }
				changeState(PICK_UP);
			}
		}else if(state==DONE){
			
		}else if(state==DEBUG){
			
			manipulator.goToPickUp2();
			System.err.println("go to pickup");
			waitUp(3000);
			manipulator.closeGripper();
			System.err.println("close gripper");
			waitUp(3000);			
	        manipulator.servoOut2((int)DATA[iterator][0],(int)DATA[iterator][1],500);
	        System.err.println("Move arm up to level "+iterator);
		    waitUp(3000);
		    manipulator.openGripper();
			System.err.println("open gripper");
		    waitUp(3000);
		    iterator++;
		    manipulator.goToPickUp2();
		    if (iterator==towerHeight){
				changeState(DONE);
			}else{
			System.err.println("go to pickup next block");
		    waitUp(3000);
			}
		    
		}
	}
	
	private void changeState(int newState){
		state=newState;
		if(state==DEPOSIT_BLOCKS){
			stateMsg.data = "DEPOSIT_BLOCKS";
		}else if (state==PICK_UP){
			stateMsg.data = "PICK_UP";
		}else if (state==PLACE){
			stateMsg.data = "PLACE";
		}else if (state==DONE){
			stateMsg.data = "DONE!";
		}else if (state==DEBUG){
			stateMsg.data = "DEBUG";
		}else if (state==LOCALIZE){
			stateMsg.data = "LOCALIZE";
		}else if (state==GO_TO_NEXT){
			stateMsg.data = "GO_TO_NEXT";
		}
		
		statePub.publish(stateMsg);
	}
	
	public void onStart(Node node) {
		this.node = node;

		System.err.println("PHASE 2 STARTED");
		
		state=INIT;
		//collectedBlocks=myCollectedBlocks;//Pass the information about the blocks we currently have
		
		//add ConstructionBlocks to structure to define the structure we want to make. 6 or 7 or 8 blocks total?
		manipulator=new Manipulator(node);		
		robotController=new PhaseTwoMotorController();
		robotController.onStart(node);
		while(!manipulator.armPublisher.hasSubscribers()){
			
		}
		
		//reset robot odometry somehow so we only have to deal with the robot's x value from this point onward. 
		
		// initialize the ROS publication to rss/state
		breakBeamSub = node.newSubscriber("rss/BreakBeam", "rss_msgs/ArmMsg");
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		stateMsg = new org.ros.message.std_msgs.String();
		
		breakBeamSub.addMessageListener(new BreakBeamListener());
		
		Thread t=new Thread(this);
		t.start();
	}
	
	public class BreakBeamListener implements MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		@Override
		public void onNewMessage(BreakBeamMsg msg) {	
			currentBreakBeamValue = msg.beamBroken;
		}
		
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void run() {
		System.err.print("Running!");
		// TODO Auto-generated method stub
		while(state!=DONE){
			step();
			Thread.yield();
		}
	}	
	

}
