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
            new InstantCommand(()->drive.tankDrive(1, 1)),
            new WaitCommand(0.5),
            new InstantCommand(()->drive.tankDrive(0.25, 1))
        );
    }
}
