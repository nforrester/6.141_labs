package MotorControl;

import java.awt.Dimension;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Observable;

import javax.swing.JFrame;

import uORCInterface.OrcController;

/**
 * <p>Whole-robot functionality for the RSSBot.</p>
 *
 * @author Yun
 * @author vona
 * @author prentice
 * @author prior TAs
 **/
public class RobotBase extends Observable {
	
	/**
	 * <p>Left motor and encoder.</p>
	 **/
	public static final int LEFT = 0;
	
	/**
	 * <p>Right motor and encoder.</p>
	 **/
	public static final int RIGHT = 1;
	
	/**
	 * <p>Orcboard port of each motor.</p>
	 **/
	public static final int[] MOTOR_PORT = {
		0, //LEFT
		1  //RIGHT
	};
	
	/**
	 * <p>Orcboard port of each encoder.</p>
	 **/
	public static final int[] ENCODER_PORT = {
		16, //LEFT
		18	//RIGHT
	};
	
	/**
	 * <p>Control period in milliseconds.</p>
	 **/
	protected static final int CONTROLLER_PERIOD_MS = 50;
	
	/**
	 * <p>The orcboard (maslab code).</p>
	 **/
	protected OrcController orc;

	
	/**
	 * <p>The most recent pwm commands to each motor (if {@link #motorsDisabled},
	 * what the pwm command would have been).</p>
	 **/
	protected double[] pwmGoal = new double[2];
	
	/**
	 * <p>Whole-robot velocity control; needs to be externally set.</p>
	 **/
	protected RobotVelocityController robotVelocityController;
	
	/**
	 * <p>Whether the motors are enabled.</p>
	 **/
	protected boolean motorsEnabled = false;
	
	/**
	 * <p>Whether we've been {@link #estop}ped.</p>
	 **/
	protected boolean estopped = false;
	
	/**
	 * <P>Timestamp of last update in seconds.</p>
	 **/
	protected double lastTimeStamp;
	
	/**
	 * <P>Total time spent in updates in seconds since robot reset.</p>
	 **/
	protected double totalSampleTime = 0;
	
	/**
	 * <p>Robot reset time in milliseconds.</p>
	 **/
	protected double startupTimeMS;
	
	/**
	 * <p>Last time received from the orcboard.</p>
	 **/
	protected long lastOrcTimeRaw;

	/**
	 * <p>Last Encoder reading from orcboard. </p>
	 **/
	protected int[] lastEnc = new int[2];
	
	/**
	 * <p>Whether logging is enabled.</p>
	 **/
	protected boolean logging = false;
	
	/**
	 * <p>Log file index.</p>
	 **/
	protected int file_idx;
	
	/**
	 * <p>Writes to the log file, if logging.</p>
	 **/
	PrintWriter logWriter;
	
	/**
	 * <p>Get the other wheel.</p>
	 *
	 * @param wheel {@link #LEFT} or {@link #RIGHT}
	 * @return the other wheel
	 **/
	public static int otherWheel(int wheel) {
		return (wheel == LEFT) ? RIGHT : LEFT;
	}
	
	/**
	 * <p>Construct a new robot.</p>
	 **/
	public RobotBase() {
		
		startupTimeMS = System.currentTimeMillis();
		
		orc = new OrcController(MOTOR_PORT);
		/*
		orc.setCacheLifetime(-1); //Disable cache for "real-time" timing
		
		for (int m = LEFT; m <= RIGHT; m++) {
			motor[m] = new Motor(orc,
					MOTOR_PORT[m], false,   //flipping is handled 
					ENCODER_PORT[m], false, //below
					Orc.PinMode.QUADPHASEFAST);
			motor[m].set(0);
		}
		*/
		
//		System.out.println("Starting Servo Loop Thread");
		Thread controlThread  = new ControlLoopThread();
		controlThread.start();
		

		RobotGraph f = new RobotGraph();
//		f.setPreferredSize(new Dimension(501,532));
//		f.setResizable(false);
//		f.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
//		f.pack();
//		f.setVisible(true);
		this.addObserver(f);
//		this.setChanged();
//		this.notifyObservers(motor);
//		this.clearChanged();
	}
	
	/**
	 * <p>Set the whole-robot velocity controller.</p>
	 *
	 * @param vc the whole-robot velocity controller
	 **/
	public void setRobotVelocityController(RobotVelocityController vc) {
		robotVelocityController = vc;
	}
	
	/**
	 * <p>Get the whole-robot velocity controller.</p>
	 *
	 * @return the whole-robot velocity controller
	 **/
	public RobotVelocityController getRobotVelocityController() {
		return robotVelocityController;
	}
	
	/**
	 * <p>Get the current pwm goal for a wheel.</p>
	 *
	 * @param wheel {@link #LEFT} or {@link #RIGHT}
	 * @return the current pwm goal for wheel, may differ from the actual current
	 * pwm due to the maslab slew code, positive meanse corresp side of robot
	 * rolls forward
	 **/
	public int getPWMGoal(int wheel) {
		
		//avoid npe on init
		if (orc == null)
			return 0;
		
		synchronized(pwmGoal) {
			return (int) pwmGoal[wheel];
		}
	}
	
	/**
	 * <p>Enable or disable the motors.</p>
	 *
	 * @param enabled whether to enable
	 **/
	public void enableMotors(boolean enabled) {
		motorsEnabled = enabled;
	}
	
	/**
	 * <p>TBD this appears to be broken. Set a motor slew period in seconds.</p>
	 *
	 * <p>This feature is actually implemented by the Maslab code for the
	 * orcboard.</p>
	 *
	 * <p>Presumably to avoid large current spikes due to rapid motor
	 * accelerations, and possibly also to aid control implementations by
	 * providing a form of acceleration limiting, the slew period determines the
	 * rate at which the actual PWM values the motors see is changed.  Adapted
	 * from the Maslab documentation for <code>Orc.motorSlewWrite()</code>:</p>
	 *
	 * <p>When motors are written, it only changes the <i>goal</i> speed and
	 * direction. The actual speed and direction change more gradually, according
	 * to the slew rate. Larger slew periods mean that the PWM changes
	 * slower.</p>
	 *
	 * @param wheel {@link #LEFT} or {@link #RIGHT}
	 * @param slewPeriod the slew period in seconds, min effective is ~0.035s,
	 * max is 8.0s.
	 **/
	public void setMotorSlewInterval(int wheel, double slewPeriod) {
		
		int slew = (int) (8.0/slewPeriod);
		
		if (slew < 1)
			slew = 1;
		
		if (slew > 255)
			slew = 255;
		
		System.err.println("TBD is this working? slew " + wheel + ": " + slew);
		
		orc.motorSlewWrite(wheel, slew);
	}
	
	/**
	 * <p>Get a raw analog reading in volts from one of the orcboard ports.</p>
	 *
	 * <p>Ports 0-19 correspond to numbered ports. (Some of these can't do analog
	 * in.)  20-25 correspond to curent-sense inputs on motors0-3 and
	 * servos0-1.</p>
	 *
	 * @param port the port number, as above
	 *
	 * @return the raw voltage reading in volts
	 **/
	public double analogRead(int port) {
		return orc.analogRead(port);
	}
	
	/**
	 * <p>Update all the controllers with the current feedback and sample time,
	 * and compute all the new motor commands {@link #pwmGoal}.</p>
	 *
	 * @param sampleTime the sample time in seconds
	 * @param encL the left encoder ticks since last update
	 * @param encR the right encoder ticks since last update, already flipped
	 **/
	protected void updateAndControl(double sampleTime, int encL, int encR) {
		
		//tell the velocity controller the new feedback & sample time
		robotVelocityController.update(sampleTime, encL, encR);
		
		//get the new pwm commands
		synchronized(pwmGoal) {
			robotVelocityController.controlStep(pwmGoal);
		}
	}
	
	/**
	 * <p>Check whether the motors are enabled.</p>
	 *
	 * @return true iff the motors are enabled
	 **/
	public boolean motorsEnabled() {
		return motorsEnabled;
	}
	
	/**
	 * <p>Estop the robot.</p>
	 **/
	public synchronized void estop() {
		estopped = true;
		enableMotors(false);
		orc.motorSet(LEFT,0);
		orc.motorSet(RIGHT,0);
		//motor[LEFT].set(0);
		//motor[RIGHT].set(0);
		stopVelocityLog();
	}
	
	/**
	 * <p>Check whether the robot has been estopped.</p>
	 *
	 * @return true iff the robot has been estopped
	 **/
	public synchronized boolean estopped() {
		return estopped;
	}
	
	
	/**
	 * <p>Start logging velocity data.</p>
	 **/
	public void startVelocityLog() {
		
		if (logging)
			return;
		
		file_idx = file_idx + 1;
		String filename = "dataLog"+file_idx+".txt";
		try{
			logWriter = new PrintWriter(new FileWriter(new File(filename)));
			logging=true;
		} catch (IOException e) {
			System.err.println("error starting velocity log" + e);
		}
	}
	
	/**
	 * <p>Stop logging velocity data.</p>
	 **/
	public void stopVelocityLog() {
		
		if (!logging)
			return;
		
		logWriter.close();
		logging=false;
	}
	
	
	/**
	 * <p>Log one line of velocity data.</p>
	 *
	 * @param s the line
	 **/
	public void log(String s) {
		if (logging)
			logWriter.println(s);
	}
	
	
	/**
	 * <p>Implements the whole-robot control loop.</p>
	 **/
	class ControlLoopThread extends Thread {
		
		/**
		 * <p>First update just reads sensors to get initial state.</p>
		 **/
		protected boolean firstUpdate = true;
		
		/**
		 * <p>Create a new control loop.</p>
		 **/
		public ControlLoopThread() {
			setDaemon(true); //Exits automatically when all other threads have died
		}
		
		/**
		 * <p>Calls {@link RobotBase.ControlLoopThread#update} at with period
		 * {@link #CONTROLLER_PERIOD_MS}.</p>
		 **/
		public void run() {
			
			while (true) {
				
				try {
					Thread.sleep(CONTROLLER_PERIOD_MS);
				} catch(InterruptedException ex) {
					//ignore
				}
				
				update();
			}
		}
		
		/**
		 * <p>Do one whole-robot control update.</p>
		 **/
		protected synchronized void update() {
			double sampleTime;
						
			//avoid npe on init
			if (robotVelocityController == null)
				return;
			
			int encLe = orc.readEncoder(LEFT);
			int encRi = orc.readEncoder(RIGHT);
			
			if(firstUpdate){
				lastEnc[LEFT] = encLe;
				lastEnc[RIGHT] = encRi;
			}
			int encL = encLe - lastEnc[LEFT];
			int encR = encRi - lastEnc[RIGHT];

			lastEnc[LEFT] = encLe;
			lastEnc[RIGHT] = encRi;

			setChanged();
			notifyObservers(new int[] {encL, -encR}); // -encR because we want forward motion.
			clearChanged();
			
			
			
			// Get the elapsed time (in 1/1000000 sec ticks)
			// since the last set of encoder readings
			long orcTimeRaw = orc.clockReadSlave(); 
			long deltaOrcTimeRaw;
			
			// Compute delta 1/1000000 sec ticks since last
			// read, handling 16-bit integer overflow
			// from ORCboard
			if (orcTimeRaw >= lastOrcTimeRaw) {
				deltaOrcTimeRaw = orcTimeRaw - lastOrcTimeRaw;
			}
			else {
				deltaOrcTimeRaw = orcTimeRaw + 65536 - lastOrcTimeRaw;
			}
			lastOrcTimeRaw = orcTimeRaw;
			sampleTime = (double)deltaOrcTimeRaw/1000000.0;
			
			// Stop here on first run (since lastOrcTimeRaw
			// will have been uninitialized)
			if (firstUpdate) {
				firstUpdate = false;
				return;
			}
			
			totalSampleTime += sampleTime;
			
			// System.err.println("encL: " + encL + "; encR: " + encR + "; sampleTime: " + sampleTime);
			
			updateAndControl(sampleTime, encL, -encR); //flip
			
			if (motorsEnabled) {  
				//send the pwm commands to the motors
				//maslab code will clamp
				orc.motorSet(LEFT, (int)pwmGoal[LEFT]);
				orc.motorSet(RIGHT, (int)-pwmGoal[RIGHT]);
				//motor[LEFT].set((int)pwmGoal[LEFT]);
				//motor[RIGHT].set((int)-pwmGoal[RIGHT]); //flip
			} else{
				//make sure the motors are stopped
				orc.motorSet(LEFT, 0);
				orc.motorSet(RIGHT, 0);
			}
			
			//Log data from the motor controllers
			log(totalSampleTime + " " + pwmGoal[LEFT] + " " + pwmGoal[RIGHT] + " " +
					robotVelocityController.computeAngularVelocity(LEFT) +" " +
					robotVelocityController.computeAngularVelocity(RIGHT) +" " +
					robotVelocityController.getDesiredAngularVelocity(LEFT) + " " +
					robotVelocityController.getDesiredAngularVelocity(RIGHT));
		}
	}
}



