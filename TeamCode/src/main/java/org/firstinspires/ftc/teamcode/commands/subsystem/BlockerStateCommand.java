package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;

public class BlockerStateCommand extends InstantCommand {
    public BlockerStateCommand(ShooterSubsystem.BlockerState state) {
        super(
                () -> Robot.getInstance().shooterSubsystem.updateBlockerState(state)
        );
    }
}