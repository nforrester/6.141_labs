import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;



public class PotentialCell {
    private final int Xindex;
    private final int Yindex;
    private final boolean isObstacle;
    private final int fieldValue;
    private final int maxX;
    private final int maxY;
    
    private HashMap<int[],Boolean> neighbors = new HashMap<int[],Boolean>();
    private HashMap<int[],Boolean> temporaryOmitNeighbors = new HashMap<int[],Boolean>();
    
    public PotentialCell(int Xindex,int Yindex,int Xmax,int Ymax,boolean isObstacle,int fieldValue) {
        this.Xindex = Xindex;
        this.Yindex = Yindex;
        this.maxX = Xmax;
        this.maxY = Ymax;
        this.isObstacle =isObstacle;
        this.fieldValue = fieldValue;
        
        ArrayList<int[]> neighbors = this.getNeighbors(this.Xindex, this.Yindex);
        for(int[] a:neighbors) {
            if(this.isValid(a[0], a[1])) {
                this.neighbors.put(a, true);
            }
        }
     
    }
    
    public void clearTemporaryNeighbors() {
        this.temporaryOmitNeighbors.clear();
    }
    
    public void addTemporaryOmit(int[] val) {
        this.temporaryOmitNeighbors.put(val, false);
    }
    
    public HashMap<int[],Boolean> getTempStats(){
        return this.temporaryOmitNeighbors;
    }
    
    public boolean checkTempOmit(int[] k) {
        Iterator it = this.temporaryOmitNeighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            if(pres[0] == k[0] && pres[1] == k[1]) {
                return true;
            }
        }
        return false;
    }
    
    public boolean isObstacle() {
        return this.isObstacle;
    }
    

    
    public int getFieldValue() {
        return this.fieldValue;
    }
    
    public int getX() {
        return this.Xindex;
    }
    
    public int getY() {
        return this.Yindex;
    }
    
    public HashMap<int[],Boolean> getStats(){
        return this.neighbors;
    }
    
    public void changeStatus(int[] k,Boolean b) {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            if(pres[0] == k[0] && pres[1] == k[1]) {
                this.neighbors.put(pres, b);
            }
        }
    }
    
    private boolean isValid(int xVal,int yVal) {
        return xVal >=0 && yVal >=0 && xVal < this.maxX && yVal < this.maxY;
    }
    
    private ArrayList<int[]> getNeighbors(int x,int y) {
        ArrayList<int[]> neighborList = new ArrayList<int[]>();
        neighborList.add(new int[]{x,y-1,1});
        neighborList.add(new int[]{x,y+1,2});
        neighborList.add(new int[]{x-1,y,3});
        neighborList.add(new int[]{x+1,y,4});
        neighborList.add(new int[]{x-1,y-1,5});
        neighborList.add(new int[]{x+1,y-1,7});
        neighborList.add(new int[]{x-1,y+1,8  });
        neighborList.add(new int[]{x+1,y+1,6});
        return neighborList;
    }
    
    public String toString() {
        return new String(this.Xindex + "," + this.Yindex + " -> " + this.fieldValue );
    }
    
    
    
    public void formatNeighbors(PotentialCell[][] cells) {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] nextNeighbor = (int[]) it.next();
            if(cells[nextNeighbor[0]][nextNeighbor[1]].isObstacle) {
                this.neighbors.put(nextNeighbor, false);
            }
        }
    }
    
    
    public void printNeighborStats() {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            System.out.println(pres[0] + "," + pres[1] + " ->" + this.neighbors.get(pres).toString());
        }
    }
    
    public void printTempNeighborStats() {
        Iterator it = this.temporaryOmitNeighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            System.out.println(pres[0] + "," + pres[1] + " ->" + this.temporaryOmitNeighbors.get(pres).toString());
        }
    }
    
    
}
