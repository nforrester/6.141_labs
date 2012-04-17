/*
 * 
 */
package Grasping;

public class Grasping {

    
    
    
    
    
    

    
    
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
