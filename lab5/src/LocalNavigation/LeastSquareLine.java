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
	private boolean line_dirty;
	private double[] line; // {a, b, c}

	public LeastSquareLine() {
		reset();
	}

	public synchronized void reset() {
		sumX = 0;
		sumY = 0;
		sumXX = 0;
		sumYY = 0;
		sumXY = 0;

		nPoints = 0;
		line_dirty = true;
		line = {};
	}

	public synchronized void addPoint(double x, double y) {
		sumX += x;
		sumY += y;
		sumXX += x * x;
		sumYY += y * y;
		sumXY += x * y;

		nPoints++;
		line_dirty = true;
	}

	public synchronized double[] getLine() {
		double[] line = {};
		if (line_dirty) {
			if (nPoints >= 2) {
				double d = sumXX * sumYY - sumXY * sumXY;
				if (d != 0) {
					double a = (sumX * sumYY - sumY * sumXY) / d;
					double b = (sumY * sumXX - sumX * sumXY) / d;
					line = {a, b, -1};
					line_dirty = false;
				}
			}
		}
		return line;
	}
}
