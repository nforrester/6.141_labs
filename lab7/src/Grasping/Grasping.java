/*
 * 
 */
package Grasping;

import LocalNavigaton.Mat;

public class Grasping {
	public static void main(String[] args){
		Object[] valOne = new Object[] { 0.0,452.0};
		Object[] valTwo = new Object[] { Math.PI/2,1645.0};

		System.out.println(new GripperController().getAngleEquivalent((long)1645));
	}

	public static double[] inverseKinematics(double x2, double z2) {
		// Please excuse the variable names. For an explanation, see:
		// https://courses.csail.mit.edu/rss/wiki/index.php?title=Team6Lab72012#Arm_control_and_inverse_kinematics

		double x0 = .07; // rough estimate, TODO: replace with measurements!!!!
		double z0 = .12; // rough estimate, TODO: replace with measurements!!!!
		Mat p0 = encodePoint2(x0, z0);
		Mat p2 = encodePoint2(x2, z2);

		double l1 = .25; // rough estimate, TODO: replace with measurements!!!!
		double l2 = .15; // rough estimate, TODO: replace with measurements!!!!

		double phi = Math.atan2(z2 - z0, x2 - x0);
		double d = Mat.l2(Mat.add(p2, Mat.mul(-1, p0)));

		double x1_ = (d*d - l2*l2 + l1*l1) / (2 * d);
		double z1_ = Math.pow(4 * d*d * l1*l1 - Math.pow(d*d - l2*l2 + l1*l1, 2), 0.5) / (2 * d);

		double phi_ = Math.atan2(z1_, x1_);

		double theta1 = phi + phi_;
		double theta2 = Math.acos((x2 - x0 - l1 * Math.cos(theta1)) / l2) - theta1;

		return new double[]{theta1, theta2};
	}

	public static Mat encodePoint2(double x, double y) {
		Mat p = new Mat(2, 1);
		p.data[0][0] = x;
		p.data[1][0] = y;
		return p
	}

	public static double[] decodePoint2(Mat p) {
		double x = p.data[0][0];
		double y = p.data[1][0];
		return new double[]{x, y};
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
