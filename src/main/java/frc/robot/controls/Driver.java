package frc.robot.controls;

import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
    driver.get(Button.A).whenPressed(new DoNothing());
  }

  public static double getLeftX(){
    return driver.get(Axis.LEFT_X);
  }
  public static double getLeftY(){
    return driver.get(Axis.LEFT_Y);
  }
  public static double getRightX(){
    return driver.get(Axis.RIGHT_X);
  }
  public static double getRightY(){
    return driver.get(Axis.RIGHT_Y);
  }

}
