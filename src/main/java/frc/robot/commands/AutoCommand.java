package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(){
        addRequirements(Robot.m_drive);
        addCommands(
            new PrintCommand("Starting auto"),
            new PathweaverCommand("ForwardPart", true, false),
            new PathweaverCommand("BackwardPart", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, false),
            new PathweaverCommand("Backward2", false, false),
            new PathweaverCommand("Forward2", false, true)
        );
    }
}
