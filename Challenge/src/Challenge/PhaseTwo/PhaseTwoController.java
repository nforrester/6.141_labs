package Challenge.PhaseTwo;

import java.util.ArrayList;

import Challenge.RobotController;
import Challenge.Waypoint;

/**
 * <p>Deals with Contructing Blocks</p>
 *
 * @author dgonz
 *
 **/

public class PhaseTwoController {
	
	public static final int INIT=0;
	public static final int DEPOSIT_BLOCKS=1;
	public static final int PICK_UP=2;
	public static final int PLACE=3;
	public static final int DONE=4;
	private int state;
	
	double x=0; //end effector x
	double y=0; //end effector y
	double theta=0;//end effector theta
	
	Structure structure;
	Manipulator manipulator;
	RobotController robotController;
	ArrayList<ConstructionBlock> collectedBlocks;
	
	public PhaseTwoController(ArrayList<ConstructionBlock> myCollectedBlocks) {
		state=INIT;
		collectedBlocks=myCollectedBlocks;//Pass the information about the blocks we currently have 
		structure=new Structure();
		
		//add ConstructionBlocks to structure to define the structure we want to make. 6 or 7 or 8 blocks total?
		manipulator=new Manipulator();
		robotController=new RobotController();
		
		//reset robot odometry somehow so we only have to deal with the robot's x value from this point onward. 
	}
	
	public void goToY(double yNew){
		manipulator.goToY(y);
	}
	
	public void goToX(double xNew){
	//	if(xNew>robotController.getX()){
	//		robotController.addWaypoint(new Waypoint(xNew,0,(short) 1));
	//	}else{
			robotController.addWaypoint(new Waypoint(xNew,0,(short) 1));
	//	}
	}

}
