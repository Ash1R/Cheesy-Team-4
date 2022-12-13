package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
    driver.get(Button.A).whenPressed(new InstantCommand(()->Robot.m_intake.toggle()));
    driver.get(Button.B).whenPressed(new InstantCommand(()->Robot.m_intake.setMotor(-0.2)));
    driver.get(Button.X).whenPressed(new InstantCommand(()->Robot.m_intake.setMotor(0.7/Constants.intake.speed)));
    driver.get(Button.RB).whenPressed(new InstantCommand(()->Robot.m_outtake.setSetpoint(Constants.outtake.openSetpoint)));
    driver.get(Button.RB).whenReleased(new InstantCommand(()->Robot.m_outtake.setSetpoint(Constants.outtake.closedSetpoint)));
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
  public static double getLeftTrigger() {
    return driver.get(Axis.LEFT_TRIGGER);
  }
}
