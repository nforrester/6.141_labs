package MotorControl;

/**
 * <p>Entry point for motor control lab.</p>
 **/
public class MotorControl {

  /**
   * <p>Entry point for the Motor Control lab.</p>
   *
   * @param args command line arguments
   **/
  public static void main(String [] args) {
    
    RobotBase robot = new RobotBase();
    
    RobotVelocityController robotVelocityController = null;

    //first task: direct pwm commanding

    robotVelocityController =
     new RobotVelocityController(new WheelVelocityControllerPWM(),
                                  new WheelVelocityControllerPWM());


    //second task: feed-forward control

//    robotVelocityController =
//      new RobotVelocityController(new WheelVelocityControllerFF(),
//                                  new WheelVelocityControllerFF());


    //third task: integral feedback control

//    robotVelocityController =
//      new RobotVelocityController(new WheelVelocityControllerI(),
//                                  new WheelVelocityControllerI());


    //final task: balanced velocity control

   // robotVelocityController = new RobotVelocityControllerBalanced();

    robot.setRobotVelocityController(robotVelocityController);
    MotorControlGUI GUI = new MotorControlGUI(robot);
    
    while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        System.out.println("Ending...");
        System.out.println(e);
        robot.enableMotors(false);
        robot.stopVelocityLog();
      }
    }
  }
}
