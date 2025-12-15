package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.BlockerStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ArtifactShootLONGCommand extends SequentialCommandGroup {
    public ArtifactShootLONGCommand() {
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
                new WaitCommand(500),
                new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
        );
    }
}
