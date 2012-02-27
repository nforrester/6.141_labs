package Servo;

import MotorControlSolution.RobotBase;
import orc.Orc;
import orc.Servo;



public class ServoControl {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		RobotBase robot = new RobotBase();
		Orc orc = Orc.makeOrc();
		System.out.println("made orc");
		SafeServo s1 = new SafeServo(orc, 0, 750, 2300, 750);
		System.out.println("created servo");
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		int pulse = 750;
		while (pulse < 2300){
			pulse += 10;
			s1.setPulseWidth(pulse);
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		s1.idle();
		
	}

}
