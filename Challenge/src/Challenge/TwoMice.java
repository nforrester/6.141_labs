package Challenge;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Controller.Type;

public class TwoMice {
	//put this in VM arguments before running
	// -Djava.library.path=dist/

	private static Controller mouseOne = null;
	private static Controller mouseTwo = null;

	private static Controller keyboard = null;

	private static final String mouseName = "DELL USB Laser Mouse";

	private static double mouseOneX = 0;
	private static double mouseOneY = 0;

	private static double mouseTwoX = 0;
	private static double mouseTwoY = 0;

	private static double mouseOneDeltaX = 0;
	private static double mouseOneDeltaY = 0;

	private static double mouseTwoDeltaX = 0;
	private static double mouseTwoDeltaY = 0;

	private static double x;
	private static double y;
	private static double theta;
	private static double TICKS_PER_METER = 17000; // needs to be measured, not guessed
	private static double DISTANCE_BETWEEN_SENSORS = .064; // meters


	public static void main(String args[]) {
		Controller[] ca = ControllerEnvironment.getDefaultEnvironment().getControllers();

		for (Controller controller : ca) {
			if (controller.getType() == Type.MOUSE) {
				if(mouseOne == null && controller.getName().trim().equals(mouseName)) {
					mouseOne = controller;
					System.out.println(mouseOne.getName());
				}
				else if(controller.getName().trim().equals(mouseName)) {
					mouseTwo = controller;
				}
			} else if (controller.getType() == Type.KEYBOARD) {
				keyboard = controller;
			}
		}

		// throw exception if mouse not present
		if (mouseOne == null ) {
			throw new NullPointerException("You don't have a mouse");
		}
		else if(keyboard == null) {
			throw new NullPointerException("You don't have a keyboard");
		}


		while (true) {
			mouseOne.poll();
			Component[] componentsOne = mouseOne.getComponents();

			for(int i=0;i< componentsOne.length;i++) {
				if(componentsOne[i].isAnalog()) {
					if(componentsOne[i].getName().equals("x")) {
						mouseOneDeltaX = (double)componentsOne[i].getPollData();
						mouseOneX += mouseOneDeltaX;
					}
					else if(componentsOne[i].getName().equals("y")) {
						mouseOneDeltaY = (double)componentsOne[i].getPollData();
						mouseOneY += mouseOneDeltaY;
					}
				}
			}

			mouseTwo.poll();
			Component[] componentsTwo = mouseTwo.getComponents();

			for(int i=0;i< componentsTwo.length;i++) {
				if(componentsTwo[i].isAnalog()) {
					if(componentsTwo[i].getName().equals("x")) {
						mouseTwoDeltaX = (double)componentsTwo[i].getPollData();
						mouseTwoX += mouseTwoDeltaX;
					}
					else if(componentsTwo[i].getName().equals("y")) {
						mouseTwoDeltaY = (double)componentsTwo[i].getPollData();
						mouseTwoY += mouseTwoDeltaY;
					}
				}
			}


			// call the update method here
			// mouseOne is left and MouseTwo is right
			update(getMouseOneDeltaX(),getMouseOneDeltaY(),getMouseTwoDeltaX(),getMouseTwoDeltaY());

			// method to print out the values of the mouse movements :)
			//System.out.println("Mouse X One -> " + getMouseOneX() + " Mouse Y One -> " + getMouseOneY() + " Mouse X Two -> " + getMouseTwoX() + " Mouse Y Two -> " + getMouseTwoY() );
			System.out.println("X -> " + x + " Y -> " + y + " theta -> " + theta);

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}


	public static double getMouseOneX() {
		return mouseOneX;
	}

	public static double getMouseOneY() {
		return -mouseOneY;
	}

	public static double getMouseTwoX() {
		return mouseTwoX;
	}

	public static double getMouseTwoY() {
		return -mouseTwoY;
	}

	public static double getMouseTwoDeltaX() {
		return mouseTwoDeltaX;
	}

	public static double getMouseTwoDeltaY() {
		return -mouseTwoDeltaY;
	}

	public static double getMouseOneDeltaX() {
		return mouseOneDeltaX;
	}

	public static double getMouseOneDeltaY() {
		return -mouseOneDeltaY;
	}

	// mouse 1 is left, mouse 2 is right
	private static void update(double dx1, double dy1, double dx2, double dy2) {
		// ticks to meters
		dx1 /= TICKS_PER_METER;
		dy1 /= TICKS_PER_METER;
		dx2 /= TICKS_PER_METER;
		dy2 /= TICKS_PER_METER;

		// mouse frame to robot start position frame
		// X axis of the robot is the Y axis of the mouse
		double dx1Robot =  dy1;
		double dy1Robot = -dx1;
		double dx2Robot =  dy2;
		double dy2Robot = -dx2;

		// useful definitions
		double yAvg  = (dy2Robot + dy1Robot) / 2;
		double xAvg  = (dx2Robot + dx1Robot) / 2;
		double xDiff =  dx2Robot - dx1Robot;
		double dTheta = xDiff / DISTANCE_BETWEEN_SENSORS;
		double thetaTravel = theta + dTheta / 2;

		// compute new odometry
		double xNew = x + Math.cos(thetaTravel) * xAvg - Math.sin(thetaTravel) * yAvg;
		double yNew = y + Math.sin(thetaTravel) * xAvg + Math.cos(thetaTravel) * yAvg;
		double thetaNew = theta + dTheta;

		// update new odometry
		x = xNew;
		y = yNew;
		theta = thetaNew;
	}
}
