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
 *      coordinateTransformation = mul(rot, trans);
 *
 *      poseOdo = encodePose(xOdo, yOdo, thetaOdo);
 *      poseRobot = mul(coordinateTransformation, poseOdo);
 *      p = decodePose(poseRobot);
 *      x = p[0];
 *      y = p[1];
 *      z = p[2];
 *
 */

package LocalNavigation;
import java.io.PrintStream;

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
	 * <p>Print a matrix</p>
	 */
	public static Mat print(Mat m) {
		return print(System.out, m);
	}

	public static Mat print(PrintStream o, Mat m) {
		o.println("(" + m.rows + " " + m.columns + ")");

		int row;
		int column;

		for (row = 0; row < m.rows; row++) {
			if (row == 0) {
				o.print("[");
			} else {
				o.print(" ");
			}
			for (column = 0; column < m.columns; column++) {
				o.print(" " + m.data[row][column]);
			}
			if (row != m.rows - 1) {
				o.println("");
			} else {
				o.println(" ]");
			}
		}

		return m;
	}

	/**
	 * <p>Copy a matrix</p>
	 */
	public static Mat copy(Mat matrix) {
		Mat copy = new Mat(matrix.rows, matrix.columns);

		int row;
		int column;

		for (row = 0; row < matrix.rows; row++) {
			for (column = 0; column < matrix.columns; column++) {
				copy.data[row][column] = matrix.data[row][column];
			}
		}

		return copy;
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
	public static Mat encodePose(double[] pose) {
		return encodePose(pose[0], pose[1], pose[2]);
	}
	
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
		theta = rerangeAngle(theta);
		double[] ret = {x, y, theta};
		return ret;
	}

	/**
	 * <p>Encode a point matrix</p>
	 */
	public static Mat encodePoint(double[] point) {
		return encodePoint(point[0], point[1]);
	}
	
	public static Mat encodePoint(double x, double y) {
		return encodePose(x, y, 0);
	}
	
	/**
	 * <p>Decode a point matrix</p>
	 */
	public static double[] decodePoint(Mat point) {
		double[] pose = decodePose(point);
		return new double[] {pose[0], pose[1]};
	}

	private static double rerangeAngle(double a) {
		while (a >= 2 * Math.PI) {
			a -= 2 * Math.PI;
		}
		while (a < 0) {
			a += 2 * Math.PI;
		}
		return a;
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
	 * <p>Subtract two matricies</p>
	 */
	public static Mat sub(Mat mA, Mat mB) {
		return add(mA, mul(-1, mB));
	}

	/**
	 * <p>Multiply two matricies</p>
	 */
	public static Mat mul(Mat mA, Mat mB) {
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

	/**
	 * <p>Multiply many matricies</p>
	 */
	public static Mat mul(Mat... matrices) {
		Mat product = matrices[0];

		for (int matrix = 1; matrix < matrices.length; matrix++) {
			product = mul(product, matrices[matrix]);
		}

		return product;
	}

	/**
	 * <p>Multiply a scalar and a matrix</p>
	 */
	public static Mat mul(double a, Mat mB) {
		Mat mC = new Mat(mB.rows, mB.columns);

		int row;
		int column;

		for (row = 0; row < mB.rows; row++) {
			for (column = 0; column < mB.columns; column++) {
				mC.data[row][column] = a * mB.data[row][column];
			}
		}

		return mC;
	}

	/**
	 * <p>Transpose a matrix</p>
	 */
	public static Mat transpose(Mat m) {
		Mat mT = new Mat(m.columns, m.rows);

		int row;
		int column;

		for (row = 0; row < m.rows; row++) {
			for (column = 0; column < m.columns; column++) {
				mT.data[column][row] = m.data[row][column];
			}
		}

		return mT;
	}

	/**
	 * <p>Dot product of two vectors</p>
	 */
	public static double dot(Mat v1, Mat v2) {
		Mat mR = mul(transpose(v1), v2);

		return mR.data[0][0];
	}

	/**
	 * <p>Find the distance between two points</p>
	 */
	public static double dist(Mat p1, Mat p2) {
		return Math.pow(Math.pow(p1.data[0][0] - p2.data[0][0], 2.0) + Math.pow(p1.data[1][0] - p2.data[1][0], 2.0), 0.5);
	}

	/**
	 * <p>Find the L2 norm</p>
	 */
	public static double l2(Mat m) {
		double sumSquares = 0;
		int row;
		int column;
		for (row = 0; row < m.rows; row++) {
			for (column = 0; column < m.columns; column++) {
				sumSquares += Math.pow(m.data[row][column], 2);
			}
		}

		return Math.pow(sumSquares, 0.5);
	}

	/**
	 * <p>Inverse of a matrix, with Gauss-Jordan elimination</p>
	 */
	public static Mat inverse(Mat matrix) {
		assert matrix.rows == matrix.columns;
		Mat inverse = eye(matrix.rows);

		int pivot;
		int row;
		double pivotValue;
		Mat eliminator;

		for (pivot = 0; pivot < matrix.rows; pivot++) {
			eliminator = zero(matrix.rows);
			pivotValue = matrix.data[pivot][pivot];
			for (row = 0; row < matrix.rows; row++) {
				eliminator.data[row][pivot] = -1 * matrix.data[row][pivot] / pivotValue;
			}
			for (row = 0; row < matrix.rows; row++) {
				eliminator.data[row][row] = 1;
			}
			eliminator.data[pivot][pivot] = 1 / pivotValue;
			matrix  = mul(eliminator, matrix);
			inverse = mul(eliminator, inverse);
		}
		return inverse;
	}
}
