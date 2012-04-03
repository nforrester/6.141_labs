
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;




public class thirdclass{
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

        
        boolean[][][] cspacenew = new boolean[cspace.length][cspace.length][cspace.length];
        
        
       /*PotentialCell[][] cells = thirdclass.makeCells(cspace, 9, 9);
        int[] target =thirdclass.findNextCell(cells, cells[0][9]);
        System.out.println("Next Target -> " + target[0] + "," + target[1] + " value -> " + cells[target[0]][target[1]].getFieldValue());
        cells[target[0]][target[1]].printNeighborStats();*/
        
        
        
        // backtracking problem checking for same number transition!!!
        thirdclass.pathFind(cspace, new int[] {2,0}, new int[] {9,9});
        
        
    }
    
    
    
    
///////////////////////////////////////////Main Methods for a 3-D Array..../////////////////////////////////////////////////////////////////////////////////////////// 

/**
* Make potential field from the
* given Cspace and the goal points.
* 
* @param cspace
*            the cspace
* @param goalx
*            the goalx
* @param goaly
*            the goaly
* @param goalTheta
*            the goal theta
* @return the int[][][]
*/
public static int[][][] makePotentialField(boolean[][][] cspace,int goalx,int goaly,int goalTheta){
//assuming that cspace is a rectangle
int[][][] array = new int[cspace.length][cspace[0].length][cspace[0][0].length];
array[goalx][goaly][goalTheta] = 0;
for(int i=0;i<array.length;i++) {
for(int j=0;j<array[i].length;j++) {
for(int k=0;k<array[i][j].length;k++) {
int absxi = Math.abs(goalx -i);
int absyj = Math.abs(goaly -j);
int abszk = Math.abs(goalTheta- k);

if(cspace[i][j][k]) {
if(absxi >= absyj && absxi >= abszk) {
array[i][j][k] = absxi;
}
else if(absyj >= absxi && absyj >= abszk) {
array[i][j][k] = absyj;
}
else array[i][j][k] = abszk;
}
else {
array[i][j][k] = -1; //setting objects to -1
}

}
}
}
return array;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




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
                        }
                    }
                    else {
                        array[i][j] =-1;
                    }
                }
            }
        }

        
        ///////////////////////////////////Print Out Values///////////////////////////////
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
    int[][] numberArray = thirdclass.twoDPath(cspace, goal[0], goal[1]);
    PotentialCell[][] potentialField = thirdclass.makeCells(numberArray);
    int[] currentPosition = start.clone();
    int currentFieldValue = potentialField[start[0]][start[1]].getFieldValue();
    
    if(currentFieldValue == -1) {
        throw new Exception("No solution Exists because the robot is on an obstacle!");
    }
    
   while(potentialField[currentPosition[0]][currentPosition[1]].getFieldValue() != 0) { //terminating condition

       int[] tempCurrentPosition = thirdclass.findNextCell(potentialField, potentialField[currentPosition[0]][currentPosition[1]]);
       
       
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
       System.out.println("moving to " + currentPosition[0] + "," + currentPosition[1] + " -> " +  potentialField[currentPosition[0]][currentPosition[1]].getFieldValue());
   }
   return wayPoints;
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
        for(PotentialCell[] a:array) {
            for(PotentialCell b: a) {
                System.out.print(((b.getFieldValue() >= 0) ? b.getFieldValue() : " " )+ ",");
            }
            System.out.println();
        }
        /////////////////////////////////////////////////////////////////////////////////////
        return array;
    }
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
/////////////////////////////////////////////////////2-Dimension///////////////////////////////////////////////////////////////////////////////      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////     
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
    
    
    

}