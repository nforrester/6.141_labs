package GlobalNavigation;

import java.util.ArrayList;

/**
 * Maps real values to discrete objects.
 * You specify several objects and where they fall on the real number line.
 * You can then query a number for either the nearest object, or the location of the nearest object.
 * You can make it a real number circle rather than a real number line by using the setWrap function.
 * setWrap should be called before adding pairs to the DoubleMap, if you're going to set call it.
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

	private boolean wrapMode = false;
	private double wrapLow;
	private double wrapHigh;

	public void setWrap(double low, double high) {
		assert low < high;
		assert low < 0;
		assert 0 < high;
		wrapMode = true;
		wrapLow  = low;
		wrapHigh = high;
	}

	private double rerange(double x) {
		if (wrapMode) {
			while (x < wrapLow) {
				x += wrapHigh - wrapLow;
			}
			while (x > wrapHigh) {
				x -= wrapHigh - wrapLow;
			}
		}
		return x;
	}

	private double rerangePos(double x) {
		if (wrapMode) {
			while (x < 0) {
				x += wrapHigh - wrapLow;
			}
			while (x > wrapHigh - wrapLow) {
				x -= wrapHigh - wrapLow;
			}
		}
		return x;
	}

	private int searchHigh(double testKey) {
		int low = 0;
		int high = pairs.size() - 1;
		int split = high / 2;

		if (pairs.get(low).k > testKey) {
			return low;
		}

		if (pairs.get(high).k < testKey) {
			return high + 1;
		}

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

		int lowStart = low;
		int highStart = high;

		boolean lowFailure = false;
		boolean highFailure = false;
		while (!lowFailure || !highFailure) {
			if (!lowFailure) {
				if (pairs.get(low).w > rerangePos(testKey - pairs.get(low).k) && pairs.get(low).w < tolerance) {
					return low;
				}
				low--;
				if (wrapMode) {
					if (low < 0) {
						low = pairs.size() - 1;
					}
					if (low == highStart || rerangePos(testKey - pairs.get(low).k) > tolerance) {
						lowFailure = true;
					}
				} else {
					if (low < 0 || testKey - pairs.get(low).k > tolerance) {
						lowFailure = true;
					}
				}
			}
			if (!highFailure) {
				if (pairs.get(high).w > rerangePos(pairs.get(high).k - testKey) && pairs.get(high).w < tolerance) {
					return high;
				}
				high++;
				if (wrapMode) {
					if (high >= pairs.size()) {
						high = 0;
					}
					if (high == lowStart || rerangePos(pairs.get(high).k - testKey) > tolerance) {
						highFailure = true;
					}
				} else {
					if (high > pairs.size() || pairs.get(high).k - testKey > tolerance) {
						highFailure = true;
					}
				}
			}
		}
		return -1;
	}

	public Maybe<Pair> get(double key, double tolerance) {
		key = rerange(key);
		int index = search(key, tolerance);
		if (index != -1) {
			return new Maybe().just(pairs.get(index));
		} else {
			return new Maybe().none();
		}
	}

	public void put(double key, V value, double width) {
		key = rerange(key);
		pairs.add(searchHigh(key), new Pair(key, value, width));
	}
}
