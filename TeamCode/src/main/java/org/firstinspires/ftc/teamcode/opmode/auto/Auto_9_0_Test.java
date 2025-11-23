package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactLowerPowerShootCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Constants;
@Disabled
@Autonomous(name = "Auto_6_0_Blue_Test")
@Configurable
public class Auto_9_0_Test extends LinearOpMode {



    public static double startX = 20.5;
    public static double startY = 123;
    public static double startHeading = 144;


    public static double shoot0X = 52.5;
    public static double shoot0Y = 103.5;
    public static double shoot0Heading = 145;


    public static double intake11X = 51;
    public static double intake11Y = 87;
    public static double intake11Heading = 180;

    public static double intake12X = 23.5;
    public static double intake12Y = 87;
    public static double intake12Heading = 180;


    public static double shoot1X = 52.5;
    public static double shoot1Y = 103.5;
    public static double shoot1Heading = 145;


    public static double intake21X = 51;
    public static double intake21Y = 60;
    public static double intake21Heading = 180;

    public static double intake22X = 17;
    public static double intake22Y = 60;
    public static double intake22Heading = 180;


    public static double shoot2X = 52.5;
    public static double shoot2Y = 103.5;
    public static double shoot2Heading = 145;


    public static double intake31X = 51;
    public static double intake31Y = 36;
    public static double intake31Heading = 180;

    public static double intake32X = 23.5;
    public static double intake32Y = 36;
    public static double intake32Heading = 180;

    public static double shoot3X = 52.5;
    public static double shoot3Y = 103.5;
    public static double shoot3Heading = 145;

    public static double intake41X = 15;
    public static double intake41Y = 41.4;
    public static double intake41Heading = 270;

    public static double intake42X = 15;
    public static double intake42Y = 12.8;
    public static double intake42Heading = 270;

    public static double shoot4X = 52.5;
    public static double shoot4Y = 103.5;
    public static double shoot4Heading = 145;

    public static double move3X = 46;
    public static double move3Y = 75;
    public static double moveHeading = 145;



    public static Path shoot0Path;
    public static Path intake11Path;
    public static Path intake12Path;
    public static Path shoot1Path;
    public static Path intake21Path;
    public static Path intake22Path;
    public static Path shoot2Path;
    public static Path intake31Path;
    public static Path intake32Path;
    public static Path shoot3Path;
    public static Path intake41Path;
    public static Path intake42Path;
    public static Path shoot4Path;
    public static Path movePath;


    public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    public void buildPaths() {
        Pose shoot0Pose = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0Heading));
        Pose intake11Pose = new Pose(intake11X, intake11Y, Math.toRadians(intake11Heading));
        Pose intake12Pose = new Pose(intake12X, intake12Y, Math.toRadians(intake12Heading));
        Pose shoot1Pose = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1Heading));
        Pose intake21Pose = new Pose(intake21X, intake21Y, Math.toRadians(intake21Heading));
        Pose intake22Pose = new Pose(intake22X, intake22Y, Math.toRadians(intake22Heading));
        Pose shoot2Pose = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2Heading));
        Pose intake31Pose = new Pose(intake31X, intake31Y, Math.toRadians(intake31Heading));
        Pose intake32Pose = new Pose(intake32X, intake32Y, Math.toRadians(intake32Heading));
        Pose shoot3Pose = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3Heading));
        Pose intake41Pose = new Pose(intake41X, intake41Y, Math.toRadians(intake41Heading));
        Pose intake42Pose = new Pose(intake42X, intake42Y, Math.toRadians(intake42Heading));
        Pose shoot4Pose = new Pose(shoot4X, shoot4Y, Math.toRadians(shoot4Heading));
        Pose movePose = new Pose(move3X, move3Y, Math.toRadians(moveHeading));



        shoot0Path = buildPath(startPose, shoot0Pose);
        intake11Path = buildPath(shoot0Pose, intake11Pose, 0.5);
        intake12Path = buildPath(intake11Pose, intake12Pose);
        shoot1Path = buildPath(intake12Pose, shoot1Pose, 0.5);
        intake21Path = buildPath(shoot1Pose, intake21Pose, 0.5);
        intake22Path = buildPath(intake21Pose, intake22Pose);
        shoot2Path = buildPath(intake22Pose, shoot2Pose, 0.5);
        intake31Path = buildPath(shoot2Pose, intake31Pose, 0.5);
        intake32Path = buildPath(intake31Pose, intake32Pose);
        shoot3Path = buildPath(intake32Pose, shoot3Pose, 0.5);
        intake41Path = buildPath(shoot3Pose, intake41Pose, 0.3);
        intake42Path = buildPath(intake41Pose, intake42Pose);
        shoot4Path = buildPath(intake42Pose, shoot4Pose, 0.5);
        movePath = buildPath(shoot4Pose, movePose);


    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        while (!isStarted() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.follower.setPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PathCommand(shoot0Path).alongWith(
                                new ArtifactInCommand()
                        ),

                        new WaitCommand(1000), // to let the launcher charge up
                        new WaitCommand(300),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),


                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),

                        new PathCommand(intake11Path),
                        new PathCommand(intake12Path),

                        new PathCommand(shoot1Path),

                        new WaitCommand(300),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),

                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),

                        new PathCommand(intake21Path),
                        new PathCommand(intake22Path),

                        new PathCommand(shoot2Path),
                        new WaitCommand(300),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),

                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),


                        new PathCommand(intake31Path),
                        new PathCommand(intake32Path),

                        new PathCommand(shoot3Path),
                        new WaitCommand(300),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),

                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),

                        new PathCommand(intake41Path),
                        new PathCommand(intake42Path),

                        new PathCommand(shoot4Path),

                        new PathCommand(movePath)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }
    }
}
