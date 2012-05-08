package Challenge.PhaseTwo;

import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Challenge.RobotController;
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
	
	private int state;
	private int towerHeight=5;
	private int iterator=1;
	
	//Measured Stuff
	public static final double BLOCK_SIZE=2.07*Manipulator.I2M;
	
	public final double[][] DATA={{600,500,0},
										{750,800,-.08},
										{900,1000,-.115},
										{1000,1150,-.135},
										{0,0,-.145}};
	
	//ROS stuff
	private Node node;
	private Publisher<org.ros.message.std_msgs.String> statePub;
	private org.ros.message.std_msgs.String stateMsg;
	
	
	//Controller Stuff
	double x=0; //end effector x
	double y=0; //end effector y
	double theta=0;//end effector theta	
	Structure structure;
	Manipulator manipulator;
	RobotController robotController;
	ArrayList<ConstructionBlock> collectedBlocks;
	
	
	
	//Command Queue.
	//ArrayList<Waypoint> myWaypoints=new ArrayList<Waypoint>();
	
	/*public PhaseTwoController(ArrayList<ConstructionBlock> myCollectedBlocks) {
		state=DEBUG;
		//collectedBlocks=myCollectedBlocks;//Pass the information about the blocks we currently have 
		structure=new Structure();
		
		//add ConstructionBlocks to structure to define the structure we want to make. 6 or 7 or 8 blocks total?
		manipulator=new Manipulator(node);
		robotController=new RobotController();
		
		//reset robot odometry somehow so we only have to deal with the robot's x value from this point onward. 
	}*/
	
	public void goToY(double yNew){
		manipulator.goToY(yNew);
	//	double xOld=x;		
	//	goToX(manipulator.A_X_B+manipulator.L_ARM*Math.cos(manipulator.a)-x);
	//	x=xOld;
		y=yNew;
		
	}
	
	public void waitUp(int val){
		try {
			Thread.sleep(val);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void goToX(double xNew){
		if(xNew>robotController.getX()){
			robotController.addWaypoint(new Waypoint(xNew,0,(short) 1));
		}else{
			robotController.addWaypoint(new Waypoint(xNew,0,(short) -1));
		}
		x=xNew;
	}
	
	public double percentToY(double val){
		return (val/100)*(manipulator.Y_MAX-manipulator.Y_MIN)+manipulator.Y_MIN;
	}
	
	//Main FSM Handles
	private void step(){
		if(state==INIT){
			
			changeState(DEPOSIT_BLOCKS);
			//changeState(DEBUG);
		
		}else if(state==DEPOSIT_BLOCKS){
			
		    //assumes that odometry has been reset at the beginning of phase 2		        
		    robotController.addWaypoint(new Waypoint(-2*15*Manipulator.I2M,0, (short)-1));
		    while(robotController.getWaypoints().size()>0){
		    }
		    changeState(LOCALIZE);
		    
		}else if(state==LOCALIZE){
			
			manipulator.goToPickUp2();
			/*
			while(true){//while(IR Sensor is unbroken)
				robotController.setTranslationalVelocity(.1);
			}*/
			robotController.setTranslationalVelocity(0);
			changeState(PICK_UP);
			
		}else if(state==PICK_UP){
			
			manipulator.closeGripper();
			robotController.addWaypoint(new Waypoint(DATA[iterator][2],0, (short)-1));
		    while(robotController.getWaypoints().size()>0){
		    }
	        manipulator.servoOut2((int)DATA[iterator][0],(int)DATA[iterator][1],500);
		    waitUp(1000);
		    changeState(PLACE);
		    
		}else if(state==PLACE){
			robotController.addWaypoint(new Waypoint((towerHeight-iterator)*BLOCK_SIZE+DATA[iterator][2],0, (short)1));
		    while(robotController.getWaypoints().size()>0){
		    }
		    manipulator.openGripper();
		    iterator++;
		}else if(state==GO_TO_NEXT){
			if (iterator==towerHeight){
				robotController.addWaypoint(new Waypoint(-.1,0, (short)-1));
			    while(robotController.getWaypoints().size()>0){
			    }
				changeState(DONE);
			}else{
				robotController.addWaypoint(new Waypoint((iterator-1)*BLOCK_SIZE+DATA[iterator][2],0, (short)-1));
			    while(robotController.getWaypoints().size()>0){
			    }
			    manipulator.goToPickUp2();
			    robotController.addWaypoint(new Waypoint((iterator-1)*BLOCK_SIZE,0, (short)-1));
			    while(robotController.getWaypoints().size()>0){
			    }
				changeState(PICK_UP);
			}
		}else if(state==DONE){
			
		}else if(state==DEBUG){
			goToY(manipulator.Y_MAX);
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.err.println("going to Ymax");
			
			goToY(percentToY(80));
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.err.println("going to 80");
			
			goToY(percentToY(60));
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.err.println("going to 60");
			
			goToY(percentToY(1));
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.err.println("going to 0");
		}
	}
	
	public void onStart(Node node) {
		this.node = node;

		System.err.println("PHASE 2 STARTED");
		
		state=INIT;
		//collectedBlocks=myCollectedBlocks;//Pass the information about the blocks we currently have 
		structure=new Structure();
		
		//add ConstructionBlocks to structure to define the structure we want to make. 6 or 7 or 8 blocks total?
		manipulator=new Manipulator(node);		
		robotController=new RobotController();
		
		goToY(percentToY(100));
		
		//reset robot odometry somehow so we only have to deal with the robot's x value from this point onward. 
		
		// initialize the ROS publication to rss/state
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		stateMsg = new org.ros.message.std_msgs.String();
		
		Thread t=new Thread(this);
		t.start();
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
		}
	}	
	

}
