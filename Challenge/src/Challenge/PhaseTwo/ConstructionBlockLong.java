package Challenge.PhaseTwo;

public class ConstructionBlockLong extends ConstructionBlock {
	
	/* For the long block, the center of one of these blocks is its origin.
	 * The long block's X and Y correspond to where its origin is. 
	 * When looking at the robot from its right side, the theta is the 
	 * 		block's angle from the robot's x axis. 
	 */

	int theta;

	public ConstructionBlockLong(double myX,double myY,int myTheta) {
		super(myX,myY);
		theta=myTheta;
	}
	
	public int getTheta(){
		return theta;
	}

}
