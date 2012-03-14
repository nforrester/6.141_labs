/**
 * 
 * @author nforrest
 *
 * Matrix operations handy for robots
 *
 * Usage example:
 *
 *      trans = translation(x, y);
 *      rot = rotation(theta);
 *      coordinateTransformation = multiply(rot, trans);
 *
 *      poseOdo = encodePose(xOdo, yOdo, thetaOdo);
 *      poseRobot = multiply(coordinateTransformation, poseOdo);
 *      p = decodePose(poseRobot);
 *      x = p[0];
 *      y = p[1];
 *      z = p[2];
 *
 */

package LocalNavigation;

public class Mat {
	public double[][] data;
	public int rows;
	public int columns;

	/**
	 * <p>Create a new uninitialized Mat object.</p>
	 */
	public Mat(int nrows, int ncolumns) {
		rows = nrows;
		columns = ncolumns;
		data = new double[rows][columns];
	}

	/**
	 * <p>The zero matrix</p>
	 */
	public static Mat zero(int size) {
		return zero(size, size);
	}

	/**
	 * <p>The zero matrix</p>
	 */
	public static Mat zero(int rows, int columns) {
		Mat z = new Mat(rows, columns);

		int row;
		int column;

		for (row = 0; row < z.rows; row++) {
			for (column = 0; column < z.columns; column++) {
				z.data[row][column] = 0;
			}
		}
		return z;
	}
	
	/**
	 * <p>The identity matrix</p>
	 */
	public static Mat eye(int size) {
		Mat i = new Mat(size, size);

		int row;
		int column;

		for (row = 0; row < i.rows; row++) {
			for (column = 0; column < i.columns; column++) {
				if (row == column) {
					i.data[row][column] = 1;
				} else {
					i.data[row][column] = 0;
				}
			}
		}
		return i;
	}
	
	/**
	 * <p>Encode a pose matrix</p>
	 */
	public static Mat encodePose(double x, double y, double theta) {
		Mat pose = new Mat(4, 1);
		pose.data[0][0] = x;
		pose.data[1][0] = y;
		pose.data[2][0] = theta;
		pose.data[3][0] = 1;
		return pose;
	}
	
	/**
	 * <p>Decode a pose matrix</p>
	 */
	public static double[] decodePose(Mat pose) {
		double x     = pose.data[0][0];
		double y     = pose.data[1][0];
		double theta = pose.data[2][0];
		double[] ret = {x, y, theta};
		return ret;
	}
	
	/**
	 * <p>A translation matrix</p>
	 */
	public static Mat translation(double x, double y) {
		Mat trans = new Mat(4, 4);
		trans.data = new double[][] {{1, 0, 0, x},
			                     {0, 1, 0, y},
               			             {0, 0, 1, 0},
	               		             {0, 0, 0, 1}};
		return trans;
	}

	/**
	 * <p>A rotation matrix</p>
	 */
	public static Mat rotation(double theta) {
		Mat rot = new Mat(4, 4);
		rot.data = new double[][] {{Math.cos(theta), -Math.sin(theta), 0, 0    },
			                   {Math.sin(theta),  Math.cos(theta), 0, 0    },
			                   {              0,                0, 1, theta},
			                   {              0,                0, 0, 1    }};
		return rot;
	}

	/**
	 * <p>Add two matricies</p>
	 */
	public static Mat add(Mat mA, Mat mB) {
		assert mA.rows    == mB.rows;
		assert mA.columns == mB.columns;

		Mat mC = new Mat(mA.rows, mA.columns);

		int row;
		int column;

		for (row = 0; row < mA.rows; row++) {
			for (column = 0; column < mA.columns; column++) {
				mC.data[row][column] = mA.data[row][column] + mB.data[row][column];
			}
		}

		return mC;
	}

	/**
	 * <p>Multiply two matricies</p>
	 */
	public static Mat multiply(Mat mA, Mat mB) {
		assert mA.columns == mB.rows;

		Mat mC = new Mat(mA.rows, mB.columns);

		int row;
		int column;
		int element;
		double total;

		for (row = 0; row < mA.rows; row++) {
			for (column = 0; column < mB.columns; column++) {
				total = 0;
				for (element = 0; element < mA.columns; element++) {
					total += mA.data[row][element] * mB.data[element][column];
				}
				mC.data[row][column] = total;
			}
		}

		return mC;
	}
}
