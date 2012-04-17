/*
 * 
 */
package Grasping;

public class Grasping {

    
    
    
    public static void main(String[] args){
    	Object[] valOne = new Object[] { 0.0,452.0};
    	Object[] valTwo = new Object[] { Math.PI/2,1645.0};
    	
    	System.out.println(new GripperController().getAngleEquivalent((long)1645));
    }
    
    

    
    
    /**
     * Calculate slope for a 
     * servo.
     * 
     * @param readingOne
     *            the reading one array
     *            consisting of {theta,PWM}
     * @param readingTwo
     *            the reading two array
     *            consisting of {theta,PWM}
     * @return the slope
     */
    public static double calculateM(Object[] readingOne, Object[] readingTwo) {
        return ((Double)readingTwo[0] - (Double)readingOne[0])/((Double)readingTwo[1] - (Double)readingOne[1]);
    }
}
