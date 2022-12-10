package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(){ //todo (maket his work)
        addRequirements(Robot.m_drive);
        addCommands(
            parallel(
                new InstantCommand(()->Robot.m_outtake.startPid(Constants.outtake.closedSetpoint)),
                new PrintCommand("Starting auto"),
                new PathweaverCommand("ForwardPart", true, false)
            ),
            new PathweaverCommand("BackwardPart", false, false),
            // new WaitCommand(100),
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
