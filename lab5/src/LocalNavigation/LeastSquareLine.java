/**
 * 
 * @author nforrest
 *
 * A least square estimator for a line
 *
 */

package LocalNavigation;

import java.lang.Exception;

public class LeastSquareLine {
	private double sumX;
	private double sumY;
	private double sumXX;
	private double sumYY;
	private double sumXY;

	private long nPoints;
	private boolean lineDirty;
	private double[] line; // {a, b, c}

	public LeastSquareLine() {
		reset();
	}

	public void reset() {
		sumX = 0;
		sumY = 0;
		sumXX = 0;
		sumYY = 0;
		sumXY = 0;

		nPoints = 0;
		lineDirty = true;
		line = new double[] {};
	}

	public void addPoint(double x, double y) {
		sumX += x;
		sumY += y;
		sumXX += x * x;
		sumYY += y * y;
		sumXY += x * y;

		nPoints++;
		lineDirty = true;
	}

	public double[] getLine() {
		if (lineDirty) {
			if (nPoints >= 2) {
				double d = sumXX * sumYY - sumXY * sumXY;
				if (d != 0) {
					double a = (sumX * sumYY - sumY * sumXY) / d;
					double b = (sumY * sumXX - sumX * sumXY) / d;
					double ln = Math.pow(a * a + b * b, 0.5);
					line = new double[] {a / ln, b / ln, -1 / ln};
					lineDirty = false;
				}
			}
		}
		return line;
	}

	public double getNPoints() {
		return nPoints;
	}

	public double getDistance(double x, double y) throws Exception {
		double[] line = getLine();
		if (line.length > 0) {
			return Math.abs(line[0] * x + line[1] * y + line[2]);
		} else {
			throw new Exception("Bad line in getDistance\n");
		}
	}
}
