package Challenge.PhaseTwo;

import java.util.ArrayList;

public class Structure {
		
	//List of 
	ArrayList<ConstructionBlock> myBlocks=new ArrayList<ConstructionBlock>();
	
	public Structure(){
		
	}
	
	public void addBlock(ConstructionBlock block){
		myBlocks.add(block);
	}
	
}
