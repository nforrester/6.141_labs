import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;


public class PotentialCell3D {
    private final int Xindex;
    private final int Yindex;
    private final int thetaType;
    private final boolean isObstacle;
    private final int fieldValue;
    private final int maxX;
    private final int maxY;
    private final int maxZ;
    
    private HashMap<int[],Boolean> neighbors = new HashMap<int[],Boolean>();
   
    
    public PotentialCell3D(int Xindex,int Yindex,int thetaType,int Xmax,int Ymax,int maxZ,boolean isObstacle,int fieldValue) {
        this.Xindex = Xindex;
        this.Yindex = Yindex;
        this.thetaType = thetaType;
        this.maxX = Xmax;
        this.maxY = Ymax;
        this.maxZ = maxZ;
        this.isObstacle =isObstacle;
        this.fieldValue = fieldValue;
        
        ArrayList<int[]> neighbors = this.getNeighbors(this.Xindex, this.Yindex, this.thetaType);
        for(int[] a:neighbors) {
            if(this.isValid(a[0], a[1],a[2])) {
                this.neighbors.put(a, true);
            }
        }
     
    }
    
    
    private boolean isValid(int xVal,int yVal,int kVal) {
        return xVal >=0 && yVal >=0 && xVal < this.maxX && yVal < this.maxY && kVal >= 0 && kVal < 8;
    }
    
    
    private ArrayList<int[]> getNeighbors(int x,int y,int k) {
        ArrayList<int[]> neighborList = new ArrayList<int[]>();
        if(thetaType == 0 || thetaType == 4) {
            neighborList.add(new int[]{x-1,y,k});
            neighborList.add(new int[]{x+1,y,k});
            neighborList.add(new int[]{x,y,k-1});
            neighborList.add(new int[]{x,y,k+1});
        }
        else if(thetaType == 1 || thetaType == 5) {
            neighborList.add(new int[]{x-1,y+1,k});
            neighborList.add(new int[]{x+1,y-1,k});
            neighborList.add(new int[]{x,y,k-1});
            neighborList.add(new int[]{x,y,k+1});
        }
        else if(thetaType == 2 || thetaType == 6) {
            neighborList.add(new int[]{x,y+1,k});
            neighborList.add(new int[]{x,y-1,k});
            neighborList.add(new int[]{x,y,k-1});
            neighborList.add(new int[]{x,y,k+1});
        }
        else if(thetaType == 3 || thetaType == 7) {
            neighborList.add(new int[]{x+1,y+1,k});
            neighborList.add(new int[]{x-1,y-1,k});
            neighborList.add(new int[]{x,y,k-1});
            neighborList.add(new int[]{x,y,k+1});
        }
        return neighborList;
    }
    
    public String toString() {
        return new String(this.Xindex + "," + this.Yindex + "," + this.thetaType + " -> " + this.fieldValue );
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
    
    public int getThetaType() {
        return this.thetaType;
    }
    
    public HashMap<int[],Boolean> getStats(){
        return this.neighbors;
    }
    
    
    public void changeStatus(int[] k,Boolean b) {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            if(pres[0] == k[0] && pres[1] == k[1] && pres[2] == k[2]) {
                this.neighbors.put(pres, b);
            }
        }
    }
    
    

    
    
    public void formatNeighbors(PotentialCell3D[][][] cells) {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] nextNeighbor = (int[]) it.next();
            if(cells[nextNeighbor[0]][nextNeighbor[1]][nextNeighbor[2]].isObstacle()) {
                this.neighbors.put(nextNeighbor, false);
            }
        }
    }
    
    
    public void printNeighborStats() {
        Iterator it = this.neighbors.keySet().iterator();
        while(it.hasNext()) {
            int[] pres = (int[]) it.next();
            System.out.println(pres[0] + "," + pres[1] + "," + pres[2] + " ->" + this.neighbors.get(pres).toString());
        }
    }
    

    
    
}
