package Servo;

import orc.Orc;
import orc.Servo;

public class SafeServo extends Servo {
	
	private int minPWM;
	private int maxPWM;
	private int pwm = -99;
	
	public SafeServo(Orc orc, int port, int min, int max, int initial) {
		super(orc, port, 0, 0, 0, 0);
		this.minPWM = min;
		this.maxPWM = max;
		this.setPulseWidth(initial);
	}

	private SafeServo(Orc orc, int port, double pos0, int usec0, double pos1,
			int usec1) {
		super(orc, port, pos0, usec0, pos1, usec1);
	}
	
	@Override
	public void setPulseWidth(int goal){
		if (goal > maxPWM) goal = maxPWM;
		if (goal < minPWM) goal = minPWM;
		if (this.pwm == -99){ // special case to handle first run
			//arm should already be at (or close) to pwm
			super.setPulseWidth(goal);
			this.pwm = goal;
		} else {
			while( goal < this.pwm){
				this.pwm -= 10;
				super.setPulseWidth(pwm);
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} while (goal > this.pwm) {
				this.pwm += 10;
				super.setPulseWidth(pwm);
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

}
