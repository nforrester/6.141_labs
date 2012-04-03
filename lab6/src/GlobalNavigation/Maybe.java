package GlobalNavigation;

/**
 * Serves the same purpose as Maybe in Haskell.
 *
 * @author nforrest
 *
 **/
public class Maybe<T> {
	boolean just;
	T value;

	public static just(T v) {
		Maybe<T> m = new Maybe<T>();
		m.just = true;
		m.value = v;
		return m;
	}

	public static none() {
		Maybe<T> m = new Maybe<T>();
		m.just = false;
		return m;
	}
}
