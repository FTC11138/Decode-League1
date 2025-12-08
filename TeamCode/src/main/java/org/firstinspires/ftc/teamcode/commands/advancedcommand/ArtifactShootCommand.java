package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.BlockerStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

public class ArtifactShootCommand extends SequentialCommandGroup {
    public
    ArtifactShootCommand() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new BlockerStateCommand(ShooterSubsystem.BlockerState.OPEN),
                                new WaitCommand(500)
                        ),
                        new InstantCommand(),
                        () -> Robot.getInstance().shooterSubsystem.blockerState != ShooterSubsystem.BlockerState.OPEN
                ),
                new StopStateCommand(ShooterSubsystem.StopState.READY),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                new WaitCommand(Constants.shootDelay),
                new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
        );
    }
}
