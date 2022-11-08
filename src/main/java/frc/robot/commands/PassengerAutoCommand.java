package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class PassengerAutoCommand extends SequentialCommandGroup {
    public PassengerAutoCommand(Drivetrain drive){
        addRequirements(drive);
        addCommands(
            new InstantCommand(()->drive.tankDrive(1, 1)),//Straight to 1st location
            new WaitCommand(0.5),//Wait until it's near/on 1st location
            new InstantCommand(()->drive.tankDrive(0.25, 1)),//Turn left to go to 2nd location
            new WaitCommand(0.5),//Wait until it's facing 2nd location
            new InstantCommand(()->drive.tankDrive(1, 1)),//Go straight to go to 2nd location
            new WaitCommand(0.25),//Wait until it's near/at 2nd location
            new InstantCommand(()->drive.tankDrive(1, 0.5)),//Turn right to go around table to 3rd location
            new WaitCommand(1.5),//Wait until it's near/at 3rd location
            new InstantCommand(()->drive.tankDrive(0, 0)),//Turn off motors (will not be in final design, just for testing 1st part)
        );
    }
}
