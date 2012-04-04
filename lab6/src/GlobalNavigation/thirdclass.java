package GlobalNavigation;

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
        
        boolean[][][] cspace3D = new boolean[8][cspace.length][cspace.length];
        for(int i=0;i < cspace3D.length;i++) {
            cspace3D[i] = testCspace.clone();
        }
        
        /*int[][][] k = thirdclass.threeDPath(cspace3D,5,5,5);
        PotentialCell3D[][][] potentialField = thirdclass.makeCells3D(k);
        int[] target = thirdclass.findNextCell3D(potentialField, potentialField[3][5][4]);
        System.out.println("Current Value -> " + potentialField[4][5][4].getX() + "," + potentialField[4][5][4].getY() + "," + potentialField[4][5][4].getThetaType() +  " value -> " + potentialField[4][5][4].getFieldValue());
        System.out.println("Next Target -> " + target[0] + "," + target[1] + "," + target[2] +  " value -> " + potentialField[target[0]][target[1]][target[2]].getFieldValue());*/
        
       /*PotentialCell[][] cells = thirdclass.makeCells(cspace, 9, 9);
        int[] target =thirdclass.findNextCell(cells, cells[0][9]);
        System.out.println("Next Target -> " + target[0] + "," + target[1] + " value -> " + cells[target[0]][target[1]].getFieldValue());
        cells[target[0]][target[1]].printNeighborStats();*/
        
        thirdclass.getWayPoints(thirdclass.pathFind3D(cspace3D, new int[] {2,3,2},  new int[] {6,6,6}));
        

        //thirdclass.pathFind(cspace, new int[] {2,0}, new int[] {9,9});
        
        
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
    
    
    
    public static ArrayList<int[]> pathFind3D(boolean[][][] cspace,int[] start,int[] goal) throws Exception {
        ArrayList<int[]> wayPoints = new ArrayList<int[]>();
        int[][][] numberArray = thirdclass.threeDPath(cspace, goal[0], goal[1], goal[2]);
        PotentialCell3D[][][] potentialField = thirdclass.makeCells3D(numberArray);
        int[] currentPosition = start.clone();
        int currentFieldValue = potentialField[start[0]][start[1]][start[2]].getFieldValue();
        
        if(currentFieldValue == -1) {
            throw new Exception("No solution Exists because the robot is on an obstacle!");
        }
        
       while(potentialField[currentPosition[0]][currentPosition[1]][currentPosition[2]].getFieldValue() != 0) { //terminating condition

           int[] tempCurrentPosition = thirdclass.findNextCell3D(potentialField, potentialField[currentPosition[0]][currentPosition[1]][currentPosition[2]]);
           
           
           if(tempCurrentPosition[0] == -1000) {
               System.err.println("No Solution as the robot is surrounded by obstacles!!!");
               return wayPoints;
           }
           
           
           if(tempCurrentPosition[0] != -1000) {
               currentPosition = tempCurrentPosition.clone(); 
           }
           // printing where the robot is headed!!
           int[] addArray = new int[] {currentPosition[0],currentPosition[1],currentPosition[2],potentialField[currentPosition[0]][currentPosition[1]][currentPosition[2]].getFieldValue()};
           wayPoints.add(addArray);
           System.out.println("moving to " + currentPosition[0] + "," + currentPosition[1] + "," + currentPosition[2]+ " -> " +  potentialField[currentPosition[0]][currentPosition[1]][currentPosition[2]].getFieldValue());
       }
       return wayPoints;
    }
        
    
    
    
  private static int[] findNextCell3D(PotentialCell3D[][][] potentialField,PotentialCell3D currentCell) throws Exception {
        
        HashMap<int[],Boolean> neighborStats = currentCell.getStats();      
        
        int currentCellFieldValue = currentCell.getFieldValue();
        if(currentCellFieldValue == -1) {
            throw new Exception("The robot is on an obstacle");
        }
        
        int[] currentCellCoordinates = new int[] {currentCell.getX(),currentCell.getY(),currentCell.getThetaType()};
        
        Iterator populate = neighborStats.keySet().iterator();
        if(!populate.hasNext()) return new int[] {-1000,-1000,-1000};
        int[] currentNextTarget = new int[] {-1000,-1000,-1000};
        
        HashMap<int[],Boolean> validPopulation = new HashMap<int[],Boolean>();
        
        while(populate.hasNext()) {
            int[] nextValue = (int[]) populate.next();
            if(neighborStats.get(nextValue)) {      //Add temporary stats check here
                validPopulation.put(nextValue, true);
            }
        }
        
        // if there are no valid directions in which we can travel, return no solution
        if(validPopulation.keySet().isEmpty()) return currentNextTarget;
        
        Iterator leastFind = validPopulation.keySet().iterator();
        int[] currentArray = (int[]) leastFind.next();
        int currentMinimum = potentialField[currentArray[0]][currentArray[1]][currentArray[2]].getFieldValue();
        currentNextTarget = currentArray.clone();
        while(leastFind.hasNext()) {
            currentArray = (int[]) leastFind.next();
            int dummyValue = potentialField[currentArray[0]][currentArray[1]][currentArray[2]].getFieldValue(); 
            if(dummyValue < currentMinimum) {
                currentMinimum = dummyValue;
                currentNextTarget = currentArray.clone();
            }
        }
        
      // if moving to cell of same value, so restrict direction
        
        if(currentCellFieldValue == currentMinimum && (validPopulation.size() > 2)){ // If the robot is not really stuck in an obstacle box withjust one pathway.
            HashMap<int[],Boolean> elementNeighbors = potentialField[currentCellCoordinates[0]][currentCellCoordinates[1]][currentCellCoordinates[2]].getStats();
            Iterator it = elementNeighbors.keySet().iterator();
            while(it.hasNext()) {
                int[] nextNeighbor = (int[]) it.next();
                potentialField[nextNeighbor[0]][nextNeighbor[1]][nextNeighbor[2]].changeStatus(new int[] {currentCellCoordinates[0],currentCellCoordinates[1],currentCellCoordinates[2]}, false);
            }
        }
        
        //if moving to a cell of Value greater than current cell, it is a local minimum.
        if(currentCellFieldValue < currentMinimum) {
            HashMap<int[],Boolean> elementNeighbors = potentialField[currentCellCoordinates[0]][currentCellCoordinates[1]][currentCellCoordinates[2]].getStats();
            Iterator it = elementNeighbors.keySet().iterator();
            while(it.hasNext()) {
                int[] nextNeighbor = (int[]) it.next();
                potentialField[nextNeighbor[0]][nextNeighbor[1]][nextNeighbor[2]].changeStatus(new int[] {currentCellCoordinates[0],currentCellCoordinates[1],currentCellCoordinates[2]}, false);
            }
        }       
        return currentNextTarget;
    }
  
  
    
    
    public static PotentialCell3D[][][] makeCells3D(int[][][] numberArray) throws Exception {
        // two dimensional no theta
        int length = numberArray.length;
        int breadth = numberArray[0].length;
        int height = numberArray[0][0].length;
        PotentialCell3D[][][] array = new PotentialCell3D[length][breadth][height];
        for(int i=0;i<array.length;i++) {
            for(int j=0;j<array[i].length;j++) { 
                for(int k=0;k<array[i][j].length;k++) {
                    if(numberArray[i][j][k] != -1) {
                        array[i][j][k] = new PotentialCell3D(i,j,k,length,breadth,height,false,numberArray[i][j][k]);              
                    }
                    else {
                        array[i][j][k] = new PotentialCell3D(i,j,k,length,breadth,height,true,numberArray[i][j][k]);  
                    }
                }
            }
        }
        ////// Change Obstacle stats as per obstacle Values////////////////////////////
        for(int i=0;i<array.length;i++) {
            for(int j=0;j<array[i].length;j++) {
                for(int k=0;k<array[i][j].length;k++) {
                    array[i][j][k].formatNeighbors(array);
                }
            }
        }        
        ////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////Print Out Values///////////////////////////////
        for(PotentialCell3D[][] a:array) {
            for(PotentialCell3D[] b: a) {
                for(PotentialCell3D c:b) {
                    System.out.print(((c.getFieldValue() >= 0) ? c.getFieldValue() : " " )+ ",");
                }
                System.out.println();
            }
            System.out.println();
        }
        /////////////////////////////////////////////////////////////////////////////////////
        return array;
    }

    
    
    public static int[][][] threeDPath(boolean[][][] cspace,int x,int y,int z) throws Exception {
        // two dimensional no theta
        int length = cspace.length;
        int breadth = cspace[0].length;
        int height = cspace[0][0].length;
        int[][][] array = new int[length][breadth][height];
        for(int i=0;i<length;i++) {
            for(int j=0;j<breadth;j++) {
                for(int k=0;k<height;k++)
                array[i][j][k] = -100;
            }
        }
        
        if(!cspace[x][y][z]) {
            throw new Exception("Goal cannot be reached as the requested goal is on an obstacle");
        }
        
        array[x][y][z] = 0;
        for(int m=0;m<length*breadth*height + 2;m++) {
            
            for(int i=0;i<array.length;i++) {
                for(int j=0;j<array[i].length;j++) {
                    for(int k=0;k<array[i][j].length;k++) {
                        if(cspace[i][j][k]) { //if the value is true
                            if(array[i][j][k] >= 0) {
                                if(is3DValid(i+1,j,k,length,breadth,height)&& array[i+1][j][k] == -100) array[i+1][j][k] = array[i][j][k]+1; 
                                if(is3DValid(i-1,j,k,length,breadth,height)&& array[i-1][j][k] == -100 ) array[i-1][j][k] = array[i][j][k]+1; 
                                if(is3DValid(i,j+1,k,length,breadth,height)&& array[i][j+1][k] == -100) array[i][j+1][k] = array[i][j][k]+ 1; 
                                if(is3DValid(i,j-1,k,length,breadth,height)&& array[i][j-1][k] == -100) array[i][j-1][k] = array[i][j][k]+ 1;
                                if(is3DValid(i,j,k+1,length,breadth,height)&& array[i][j][k+1] == -100) array[i][j][k+1] = array[i][j][k]+ 1; 
                                if(is3DValid(i,j,k-1,length,breadth,height)&& array[i][j][k-1] == -100) array[i][j][k-1] = array[i][j][k]+ 1;
                            }
                        }
                        else {
                            array[i][j][k] =-1;
                        }
                    }
                }
            }

        }

        
        ///////////////////////////////////Print Out Values///////////////////////////////
       /* for(int[][] a:array) {
            for(int[] b: a) {
                for(int c:b) {
                System.out.print(((c >= 0) ? c : " " )+ ",");
                }
                System.out.println();
            }
            System.out.println();
        }*/
        /////////////////////////////////////////////////////////////////////////////////////
        return array;
    }
    
    
    
    private static boolean is3DValid(int xVal,int yVal,int zVal,int maxX,int maxY,int maxZ) {
        return xVal >=0 && yVal >=0 && zVal >=0 && xVal < maxX && yVal < maxY && zVal < maxZ;
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
       //System.err.println("moving to " + currentPosition[0] + "," + currentPosition[1] + " -> " +  potentialField[currentPosition[0]][currentPosition[1]].getFieldValue());
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
