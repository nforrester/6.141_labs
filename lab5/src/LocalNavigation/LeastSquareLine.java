/**
 * 
 * @author nforrest
 *
 * A least square estimator for a line
 *
 */

package LocalNavigation;

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
		double[] line = {};
		if (lineDirty) {
			if (nPoints >= 2) {
				double d = sumXX * sumYY - sumXY * sumXY;
				if (d != 0) {
					double a = (sumX * sumYY - sumY * sumXY) / d;
					double b = (sumY * sumXX - sumX * sumXY) / d;
					line = new double[] {a, b, -1};
					lineDirty = false;
				}
			}
		}
		return line;
	}

	public double getNPoints() {
		return nPoints;
	}

	public double getDistance(double x, double y) {
		double[] line = getLine();
		if (line.length > 0) {
			return Math.abs(line[0] * x + line[1] * y + line[2]);
		} else {
			return -1;
		}
	}
}
