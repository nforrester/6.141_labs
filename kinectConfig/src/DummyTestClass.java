
public class DummyTestClass {
	
	public static void main(String[] args){
		float[] hsb = java.awt.Color.RGBtoHSB(210,133,63, null);
		System.out.println("Hue -> " + hsb[0]);
		System.out.println("Sat -> " + hsb[1]);
		System.out.println("Brig -> " + hsb[2]);
	}

}
