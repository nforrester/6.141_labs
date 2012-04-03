package GlobalNavigation;

import java.util.ArrayList;

/**
 * Maps real values to discrete objects.
 * You specify several objects and where they fall on the real number line.
 * You can then query a number for either the nearest object, or the location of the nearest object.
 *
 * @author nforrest
 *
 **/
public class DoubleMap<V> {
	public class Pair {
		public double k;
		public V v;

		public double w; // width is measured from the center, like radius, not diameter

		public Pair(double key, V value, double width) {
			k = key;
			v = value;

			w = width;
		}
	}

	private ArrayList<Pair> pairs = new ArrayList<Pair>();

	private int searchHigh(double testKey) {
		int low = 0;
		int high = pairs.size() - 1;
		int split = high / 2;

		while (low + 1 < high) {
			if (pairs.get(split).k < testKey) {
				low = split;
			} else {
				high = split;
			}
			split = (high - low) / 2 + low;
		}

		return high;
	}

	private int search(double testKey, double tolerance) {
		int low = 0;
		int high = pairs.size() - 1;
		int split = high / 2;

		while (low + 1 < high) {
			if (pairs.get(split).k < testKey) {
				low = split;
			} else {
				high = split;
			}
			split = (high - low) / 2 + low;
		}

		boolean lowFailure = false;
		boolean highFailure = false;
		while (!lowFailure || !highFailure) {
			if (!lowFailure) {
				if (pairs.get(low).w > testKey - pairs.get(low).k && pairs.get(low).w < tolerance) {
					return low;
				}
				low--;
				if (low < 0 || testKey - pairs.get(low).k > tolerance) {
					lowFailure = true;
				}
			}
			if (!highFailure) {
				if (pairs.get(high).w > pairs.get(high).k - testKey && pairs.get(high).w < tolerance) {
					return high;
				}
				high++;
				if (high > pairs.size() || pairs.get(high).k - testKey > tolerance) {
					highFailure = true;
				}
			}
		}
		return -1;
	}

	public Pair getP(double key, double tolerance) {
		return pairs.get(search(key, tolerance));
	}

	public double getK(double key, double tolerance) {
		return getP(key, tolerance).k;
	}

	public V getV(double key, double tolerance) {
		return getP(key, tolerance).v;
	}

	public void put(double key, V value, double width) {
		pairs.add(searchHigh(key), new Pair(key, value, width));
	}
}
