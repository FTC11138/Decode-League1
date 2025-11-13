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

@Autonomous(name = "Auto_6_0_Blue_Test")
@Configurable
public class Auto_9_0_Test extends LinearOpMode {



    public static double startX = 20.5;
    public static double startY = 123;
    public static double startHeading = 144;


    public static double shoot0X = 52.5;
    public static double shoot0Y = 103.5;
    public static double shoot0Heading = 149;


    public static double intake11X = 43;
    public static double intake11Y = 84;
    public static double intake11Heading = 180;

    public static double intake12X = 23;
    public static double intake12Y = 84;
    public static double intake12Heading = 180;


    public static double shoot1X = 52.5;
    public static double shoot1Y = 103.5;
    public static double shoot1Heading = 150;


    public static double intake2X = 19;
    public static double intake2Y = 60;
    public static double intake2Heading = 180;


    public static double shoot2X = 52.5;
    public static double shoot2Y = 103.5;
    public static double shoot2Heading = 150;


    public static double intake3X = 20;
    public static double intake3Y = 35.6;
    public static double intake3Heading = 180;

    public static double shoot3X = 52.5;
    public static double shoot3Y = 103.5;
    public static double shoot3Heading = 150;

    public static double move3X = 60;
    public static double move3Y = 108;
    public static double moveHeading = 145;




    // control points for intaking
    public static double control1X = 80;
    public static double control1Y = 84;

    public static double control2X = 69;
    public static double control2Y = 76;
    public static double control22X = 79;
    public static double control22Y = 56;

    public static double control3X = 89.7;
    public static double control3Y = 31.5;
    public static double control32X = 66.5;
    public static double control32Y = 34;


    public static Path shoot0Path;
    public static Path intake11Path;
    public static Path intake12Path;
    public static Path shoot1Path;
    public static Path intake2Path;
    public static Path shoot2Path;
    public static Path intake3Path;
    public static Path shoot3Path;
    public static Path movePath;


    public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    public void buildPaths() {
        Pose shoot0Pose = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0Heading));
        Pose intake11Pose = new Pose(intake11X, intake11Y, Math.toRadians(intake11Heading));
        Pose intake12Pose = new Pose(intake12X, intake12Y, Math.toRadians(intake12Heading));
        Pose shoot1Pose = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1Heading));
        Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Heading));
        Pose shoot2Pose = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2Heading));
        Pose intake3Pose = new Pose(intake3X, intake3Y, Math.toRadians(intake3Heading));
        Pose shoot3Pose = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3Heading));
        Pose movePose = new Pose(move3X, move3Y, Math.toRadians(moveHeading));



        Pose intakeControl1 = new Pose(control1X, control1Y);

        Pose intakeControl2 = new Pose(control2X, control2Y);
        Pose intakeControl22 = new Pose(control22X, control22Y);

        Pose intakeControl3 = new Pose(control3X, control3Y);
        Pose intakeControl32 = new Pose(control32X, control32Y);


        shoot0Path = buildPath(startPose, shoot0Pose);
        intake11Path = buildPath(shoot0Pose, intake11Pose, 0.5);
        intake12Path = buildPath(intake11Pose, intake12Pose);
        shoot1Path = buildPath(intake12Pose, shoot1Pose);
        intake2Path = buildCurve(shoot1Pose, intake2Pose, intakeControl2, intakeControl22);
        shoot2Path = buildPath(intake2Pose, shoot2Pose, 0.5);
        intake3Path = buildCurve(shoot2Pose, intake3Pose, intakeControl3, intakeControl32);
        shoot3Path = buildPath(intake3Pose, shoot3Pose, 0.5);
        movePath = buildPath(shoot3Pose, movePose);


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

                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),


                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),

                        new PathCommand(intake11Path),
                        new PathCommand(intake12Path),

                        new PathCommand(shoot1Path)

//                        new ArtifactShootCommand(),
//                        new WaitCommand(400),
//                        new ArtifactLowerPowerShootCommand(),
//                        new WaitCommand(800),
//                        new ArtifactShootCommand(),
//                        new ArtifactInCommand(),
//
//                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
//
//                        new PathCommand(intake2Path),
//
//                        new PathCommand(shoot2Path),
//
//                        new ArtifactShootCommand(),
//                        new WaitCommand(400),
//                        new ArtifactLowerPowerShootCommand(),
//                        new WaitCommand(800),
//                        new ArtifactShootCommand(),
//                        new ArtifactInCommand(),
//
//                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
//
//
//                        new PathCommand(intake3Path),
//
//                        new PathCommand(shoot3Path),
//
//                        new ArtifactShootCommand(),
//                        new WaitCommand(400),
//                        new ArtifactLowerPowerShootCommand(),
//                        new WaitCommand(800),
//                        new ArtifactShootCommand(),
//                        new ArtifactInCommand(),
//
//                        new PathCommand(movePath)
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
