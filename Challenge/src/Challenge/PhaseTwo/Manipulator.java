package Challenge.PhaseTwo;

import java.util.ArrayList;

import Challenge.Waypoint;

/**
 * <p>Manipulator Controller</p>
 *
 * @author dgonz
 *
 **/

public class Manipulator {
	
	//Some definitions and variables
	
	//Constants
	public static final double O_Z_A=5;  //Height from ground to Robot origin. To measure in m
	public static final double A_Z_B=2;  //Height from Robot origin to Arm. To measure in m
	public static final double A_X_B=2;  //Length from Robot origin to base of Roll Bar. To measure in m
	public static final double L_ARM=5;  //Length of Arm. To measure in m
	public static final double L_WRIST=1;  //Length of Wrist. to measure in m
	
	public static final short HAND_PORT=4;  //Servo port for gripper
	public static final int HAND_BIGOPEN=1000;  //Servo value for gripper full open
	public static final int HAND_SMALLOPEN=800;  //Servo value for gripper open but still with IR sensor working
	public static final int HAND_HOLD=500;  //Servo value for max gripping force

	public static final short WRIST_PORT=3;  //Servo port for wrist	
	public static final int WRIST_90=500;  //various exact angles
	public static final int WRIST_45=825;
	public static final int WRIST_0=1350;
	public static final int WRIST_MINUS45=1650;
	public static final int WRIST_MINUS90=2125;
	public static final double M_WRIST=-9.0556; //degrees to servo value slope
	public static final int B_WRIST=1290;  //degrees to servo value y-intercept
	
	public static final short ARM_PORT=2;  //Servo port for arm
	public static final int ARM_90=1960;  //various exact angles
	public static final int ARM_45=1600;
	public static final int ARM_0=1250;
	public static final int ARM_MINUS45=825;
	public static final double M_ARM=8.5111;  //degrees to servo value slope
	public static final int B_ARM=1211;  //degrees to servo value y-intercept
	
	
	//state variables
	double w=0; //Wrist Angle
	double a=0; //Arm Angle
	
	//Block list
	ArrayList<Waypoint> myWaypoints=new ArrayList<Waypoint>();
	public Manipulator(){
		goToPose(0,L_ARM+O_Z_A+A_Z_B,w);
	}
	public void servoOut(short port,int value){
		//Implement something involving ROS
	}
	
	public int degToServo(double theta, double m, int b){
		return (int) (b+theta*m);
	}
	
	public void goToPickUp(){
		goToY(1);
		servoOut(HAND_PORT,HAND_BIGOPEN);
	}
	
	public void goToY(double y){
		a=Math.asin((y-O_Z_A-A_Z_B*Math.cos(2.5*Math.PI/180.0))/L_ARM);
		w=-a;
		servoOut(ARM_PORT,degToServo(a,M_ARM,B_ARM));
		servoOut(WRIST_PORT,degToServo(w,M_WRIST,B_WRIST));
	}

	
	public void goToPose(double x, double y, double t){
		goToY(y);
		servoOut(WRIST_PORT,degToServo(t,M_WRIST,B_WRIST));
		goToX(x);
	}
}
