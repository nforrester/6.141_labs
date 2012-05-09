package Challenge;

import LocalNavigation.Mat;
import GlobalNavigation.PotentialCell;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

public class PotentialField{
    private int[][] returnfield;
    
    public static void main(String[] args) throws Exception {
        boolean[][] cspace = {{true,true,true,true,true,true,false,true,true,true},
                                {false,false,false,true,false,false,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,false,true,true,true,false,true,false,true},
                                {true,true,true,true,true,true,true,true,false,true}};

       
        boolean[][] testCspace = {{true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,false,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,false,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true},
                {true,true,true,true,true,true,true,true,true,true}};
    }
    
    public static ArrayList<int[]> getWayPoints(ArrayList<int[]> intialWayPoints){
        ArrayList<int[]> reduceOne = new ArrayList<int[]>();
        ArrayList<int[]> reduceTwo = new ArrayList<int[]>();
        ArrayList<int[]> reduceThree = new ArrayList<int[]>();
        ArrayList<int[]> reduceFour = new ArrayList<int[]>();
        ArrayList<int[]> checkReplicas = new ArrayList<int[]>();

        for(int[] b:intialWayPoints) { //reducing redundant data like swithching between points
            if(checkReplicas.size() == 0) {
                reduceOne.add(new int[] {b[0],b[1],b[2]});
                checkReplicas.add(b);
            }
            boolean matchFound = false;
            for(int[] c: checkReplicas) {
                if(c[0] == b[0] && c[1] == b[1] && c[2] == b[2] && c[3] == b[3]) {
                    matchFound = true;
                }
            }
            if(!matchFound) {
                reduceOne.add(new int[] {b[0],b[1]});
                checkReplicas.add(b);
            }
        }
        
        checkReplicas = new ArrayList<int[]>(); //resetting replicas
            for(int[] b: reduceOne) { //reducing angles rotations and making them implicit
                if(checkReplicas.size() == 0) {
                    reduceTwo.add(new int[] {b[0],b[1]});
                    checkReplicas.add(b);
                }
                boolean matchFound = false;
                for(int[] c: checkReplicas) {
                    if(c[0] == b[0] && c[1] == b[1]) {
                        matchFound = true;
                    }
                }
                if(!matchFound) {
                    reduceTwo.add(new int[] {b[0],b[1]});
                    checkReplicas.add(b);
                }
            }

            // Need to make Long straight paths as just two points...
            int currentXValue = -100;
            int currentYValue = -100;
            for(int[] a: reduceTwo) {
                
            }
            
            

            ArrayList<int[]> printValues = reduceTwo;
            System.out.println("reduced stuff"+ reduceTwo.size());
            for(int[] a: printValues) {
                System.out.println(a[0] + "," + a[1]);
            }

            return reduceOne;
        }
    

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
/////////////////////////////////////////////////////2-Dimension///////////////////////////////////////////////////////////////////////////////      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    
    public static int[][] twoDPath(boolean[][] cspace,int x,int y) throws Exception {
        // two dimensional no theta
        int length = cspace.length;
        int breadth = cspace[0].length;
        int[][] array = new int[length][breadth];
        for(int i=0;i<length;i++) {
            for(int j=0;j<breadth;j++) {
                array[i][j] = -100;
            }
        }
        
        if(!cspace[x][y]) {
            throw new Exception("Goal cannot be reached as the requested goal is on an obstacle");
        }
        array[x][y] = 0;
        for(int m=0;m<length*breadth + 2;m++) {
            for(int i=0;i<array.length;i++) {
                for(int j=0;j<array[i].length;j++) {
                    if(cspace[i][j]) { //if the value is true
                        if(array[i][j] >= 0) {
                            if(isValid(i+1,j,length,breadth)&& array[i+1][j] == -100) array[i+1][j] = array[i][j]+1; 
                            if(isValid(i-1,j,length,breadth)&& array[i-1][j] == -100 ) array[i-1][j] = array[i][j]+1; 
                            if(isValid(i,j+1,length,breadth)&& array[i][j+1] == -100) array[i][j+1] = array[i][j]+ 1; 
                            if(isValid(i,j-1,length,breadth)&& array[i][j-1] == -100) array[i][j-1] = array[i][j]+ 1;

                            if(isValid(i+1,j+1,length,breadth)&& array[i+1][j+1] == -100) array[i+1][j+1] = array[i][j]+1; 
                            if(isValid(i-1,j+1,length,breadth)&& array[i-1][j+1] == -100) array[i-1][j+1] = array[i][j]+1; 
                            if(isValid(i-1,j-1,length,breadth)&& array[i-1][j-1] == -100) array[i-1][j-1] = array[i][j]+1; 
                            if(isValid(i+1,j-1,length,breadth)&& array[i+1][j-1] == -100) array[i+1][j-1] = array[i][j]+1; 
                        }
                    }
                    else {
                        array[i][j] =-1;
                    }
                }
            }
        }

        
        ///////////////////////////////////Print Out Values///////////////////////////////
        for (int i = length - 1; i >= 0; i--) {
            for (int j = 0; j < length; j++) {
                System.err.print(((array[j][i] >= 0) ? ((array[j][i] > 9)? array[j][i] : (" " + array[j][i])) : " ." ));
            }
            System.err.println();
        }
        /*for(int[] a:array) {
            for(int b: a) {
                System.out.print(((b >= 0) ? b : " " )+ ",");
            }
            System.out.println();
        }*/
        /////////////////////////////////////////////////////////////////////////////////////
        return array;
    }
    
    
    
    private static boolean isValid(int xVal,int yVal,int maxX,int maxY) {
        return xVal >=0 && yVal >=0 && xVal < maxX && yVal < maxY;
    }
    
    
    
    
    
    
public static ArrayList<int[]> pathFind(boolean[][] cspace,int[] start,int[] goal) throws Exception {
    ArrayList<int[]> wayPoints = new ArrayList<int[]>();
    int[][] numberArray = PotentialField.twoDPath(cspace, goal[0], goal[1]);
    PotentialCell[][] potentialField = PotentialField.makeCells(numberArray);
    int[] currentPosition = start.clone();
    int currentFieldValue = potentialField[start[0]][start[1]].getFieldValue();
    
    if(currentFieldValue == -1) {
        throw new Exception("No solution Exists because the robot is on an obstacle!");
    }
    
   while(potentialField[currentPosition[0]][currentPosition[1]].getFieldValue() != 0) { //terminating condition

       int[] tempCurrentPosition = PotentialField.findNextCell(potentialField, potentialField[currentPosition[0]][currentPosition[1]]);
       
       
       if(tempCurrentPosition[0] == -1000 && potentialField[currentPosition[0]][currentPosition[1]].getTempStats().isEmpty()) {
           //cspace[currentPosition[0]][currentPosition[1]] = false; //newly added line
           //pathFind(cspace,start,goal);      //newly added line
           System.err.println("No Solution as the robot is surrounded by obstacles!!!");
           return wayPoints;
       }
       
       else if(tempCurrentPosition[0] == -1000 && !potentialField[currentPosition[0]][currentPosition[1]].getTempStats().isEmpty()) {
           potentialField[currentPosition[0]][currentPosition[1]].clearTemporaryNeighbors(); //clear Neighbors if end reached!
       }
       
       if(tempCurrentPosition[0] != -1000) {
           currentPosition = tempCurrentPosition.clone(); 
       }
       // printing where the robot is headed!!
       wayPoints.add(currentPosition);
       //System.err.println("moving to " + currentPosition[0] + "," + currentPosition[1] + " -> " +  potentialField[currentPosition[0]][currentPosition[1]].getFieldValue());
   }
   return wayPoints;
}
    
	public static ArrayList<Waypoint> findWaypoints(CSpace cspace, Mat start, Mat end) {
		double[] startA = Mat.decodePoint(start);
		double[] endA = Mat.decodePoint(end);
		return findWaypoints(cspace, startA[0], startA[1], endA[0], endA[1]);
	}

	public static ArrayList<Waypoint> findWaypoints(CSpace cspace, double startX, double startY, double endX, double endY) {
		float mazeSize = Math.max((float)cspace.xMax - (float)cspace.xMin, (float)cspace.yMax - (float)cspace.yMin);
		int nCells = 60;

		int startXi = (int)Math.round((startX - (float)cspace.xMin) / mazeSize * nCells);
		int startYi = (int)Math.round((startY - (float)cspace.yMin) / mazeSize * nCells);
		int endXi   = (int)Math.round((endX   - (float)cspace.xMin) / mazeSize * nCells);
		int endYi   = (int)Math.round((endY   - (float)cspace.yMin) / mazeSize * nCells);

		ArrayList<int[]> intWaypoints = new ArrayList<int[]>();
		try {
			intWaypoints = pathFind(cspace.getOccupancyGrid(nCells), new int[] {startXi, startYi}, new int[] {endXi, endYi});
		} catch (Exception e) {
			e.printStackTrace();
			System.exit(1);
		}

		ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
		for (int[] pt: intWaypoints) {
			float wpX = (float)pt[0] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)cspace.xMin - (float)startX;
			float wpY = (float)pt[1] * mazeSize / (float)nCells + mazeSize / (float)nCells / 2 + (float)cspace.yMin - (float)startY;
			waypoints.add(new Waypoint(wpX, wpY, (short) 1));
		}

		return waypoints;
	}
        
   
  private static int[] findNextCell(PotentialCell[][] potentialField,PotentialCell currentCell) {
        
        HashMap<int[],Boolean> neighborStats = currentCell.getStats();      
        
        int currentCellFieldValue = currentCell.getFieldValue();
        int[] currentCellCoordinates = new int[] {currentCell.getX(),currentCell.getY()};
        
        Iterator populate = neighborStats.keySet().iterator();
        if(!populate.hasNext()) return new int[] {-1000,-1000};
        int[] currentNextTarget = new int[] {-1000,-1000};
        
        HashMap<int[],Boolean> validPopulation = new HashMap<int[],Boolean>();
        
        while(populate.hasNext()) {
            int[] nextValue = (int[]) populate.next();
            if(neighborStats.get(nextValue) && !currentCell.checkTempOmit(nextValue)) {      //Add temporary stats check here
                validPopulation.put(nextValue, true);
            }
        }
        
        if(validPopulation.keySet().isEmpty()) return currentNextTarget;
        
        Iterator leastFind = validPopulation.keySet().iterator();
        int[] currentArray = (int[]) leastFind.next();
        int currentMinimum = potentialField[currentArray[0]][currentArray[1]].getFieldValue();
        currentNextTarget = currentArray.clone();
        while(leastFind.hasNext()) {
            currentArray = (int[]) leastFind.next();
            int dummyValue = potentialField[currentArray[0]][currentArray[1]].getFieldValue(); 
            if(dummyValue < currentMinimum) {
                currentMinimum = dummyValue;
                currentNextTarget = currentArray.clone();
            }
        }
        
      // if moving to cell of same value, so restrict direction
        if(currentCellFieldValue == currentMinimum && (validPopulation.size() <= 2)) { 
            potentialField[currentNextTarget[0]][currentNextTarget[1]].addTemporaryOmit(currentCellCoordinates);
        }
        else if(currentCellFieldValue == currentMinimum && (validPopulation.size() > 2)){ // If the robot is not really stuck in an obstacle box withjust one pathway.
            HashMap<int[],Boolean> elementNeighbors = potentialField[currentCellCoordinates[0]][currentCellCoordinates[1]].getStats();
            Iterator it = elementNeighbors.keySet().iterator();
            while(it.hasNext()) {
                int[] nextNeighbor = (int[]) it.next();
                potentialField[nextNeighbor[0]][nextNeighbor[1]].changeStatus(new int[] {currentCellCoordinates[0],currentCellCoordinates[1]}, false);
            }
        }
        
        //if moving to a cell of Value greater than current cell, it is a local minimum.
        if(currentCellFieldValue < currentMinimum) {
            HashMap<int[],Boolean> elementNeighbors = potentialField[currentCellCoordinates[0]][currentCellCoordinates[1]].getStats();
            Iterator it = elementNeighbors.keySet().iterator();
            while(it.hasNext()) {
                int[] nextNeighbor = (int[]) it.next();
                potentialField[nextNeighbor[0]][nextNeighbor[1]].changeStatus(new int[] {currentCellCoordinates[0],currentCellCoordinates[1]}, false);
            }
        }       
        return currentNextTarget;
    }
    
 
    
    
    
    public static PotentialCell[][] makeCells(int[][] numberArray) throws Exception {
        // two dimensional no theta
        int length = numberArray.length;
        int breadth = numberArray[0].length;
        PotentialCell[][] array = new PotentialCell[length][breadth];
        for(int i=0;i<array.length;i++) {
            for(int j=0;j<array[i].length;j++) { 
                if(numberArray[i][j] != -1) {
                    array[i][j] = new PotentialCell(i,j,length,breadth,false,numberArray[i][j]);              
                }
                else {
                    array[i][j] = new PotentialCell(i,j,length,breadth,true,numberArray[i][j]);  
                }
            }
        }
        
        ////// Change Obstacle stats as per obstacle Values////////////////////////////
        for(int i=0;i<array.length;i++) {
            for(int j=0;j<array[i].length;j++) {
                array[i][j].formatNeighbors(array);
            }
        }        
        ////////////////////////////////////////////////////////////////////////////////
        
        ///////////////////////////////////Print Out Values///////////////////////////////
        /*for(PotentialCell[] a:array) {
            for(PotentialCell b: a) {
                System.err.print(((b.getFieldValue() >= 0) ? b.getFieldValue() : " " )+ ",");
            }
            System.err.println();
        }*/
        /////////////////////////////////////////////////////////////////////////////////////
        return array;
    }
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
/////////////////////////////////////////////////////2-Dimension///////////////////////////////////////////////////////////////////////////////      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
    
    
    

}
