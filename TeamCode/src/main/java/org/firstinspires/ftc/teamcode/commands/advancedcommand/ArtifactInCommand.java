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

public class ArtifactInCommand extends SequentialCommandGroup {
    public
    ArtifactInCommand() {
        super(
                new BlockerStateCommand(ShooterSubsystem.BlockerState.BLOCKING),
                new WaitCommand(100),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                new StopStateCommand(ShooterSubsystem.StopState.READYSLOW),
                new ShooterStateCommand(ShooterSubsystem.ShootState.SHOOT)

        );
    }
}
