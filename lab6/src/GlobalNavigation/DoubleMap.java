package GlobalNavigation;

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

		public Pair(double key, V value) {
			k = key;
			v = value;
		}
	}

	// search modes
	private final int LOWER   = 0;
	private final int HIGHER  = 1;
	private final int CLOSEST = 2;

	private ArrayList<Pair> pairs = new ArrayList<Pair>();

	private int search(double testKey, int mode) {
		int low = 0;
		int high = pairs.size() - 1;
		int split = high / 2;

		while (low + 1 < high) {
			if (pairs.get(split) < testKey) {
				low = split;
			} else {
				high = split;
			}
			split = (high - low) / 2 + low;
		}

		if (mode == LOWER) {
			return low;
		} else if (mode == HIGHER) {
			return high;
		} else if (mode == CLOSEST) {
			if (testKey - pairs.get(low) < pairs.get(high) - testKey) {
				return low;
			} else {
				return high;
			}
		} else {
			return -1;
		}
	}

	public Pair getP(double key) {
		return pairs.get(search(key, CLOSEST));
	}

	public double getK(double key) {
		return getP(key).k;
	}

	public V getV(double key) {
		return getP(key).v;
	}

	public void put(double key, V value) {
		pairs.add(new Pair(key, value), search(key, HIGHER));
	}
}
