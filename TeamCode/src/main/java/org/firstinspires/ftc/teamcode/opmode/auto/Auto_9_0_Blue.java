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
import org.firstinspires.ftc.teamcode.commands.advancedcommand.EverythingStopCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@Autonomous(name = "Auto_6_0_Blue :(")
@Configurable
public class Auto_9_0_Blue extends LinearOpMode {



    public static double startX = 20.5;
    public static double startY = 123;
    public static double startHeading = 144;


    public static double shoot0X = 43;
    public static double shoot0Y = 108;
    public static double shoot0Heading = 140;


    public static double intake1X = 15.8;
    public static double intake1Y = 60;
    public static double intake1Heading = 180;

    public static double gateOpen1X = 15.8;
    public static double gateOpen1Y = 80;
    public static double gateOpen1Heading = 90;


    public static double shoot1X = 43;
    public static double shoot1Y = 108;
    public static double shoot1Heading = 140;


    public static double intake2X = 18;
    public static double intake2Y = 84;
    public static double intake2Heading = 180;


    public static double shoot2X = 43;
    public static double shoot2Y = 108;
    public static double shoot2Heading = 140;


    public static double intake3X = 16.9;
    public static double intake3Y = 35.7;
    public static double intake3Heading = 180;

    public static double shoot3X = 43;
    public static double shoot3Y = 108;
    public static double shoot3Heading = 140;
//
//    public static double intake4X = 12.7;
//    public static double intake4Y = 11.4;
//    public static double intake4Heading = 200;
//
//    public static double shoot4X = 43;
//    public static double shoot4Y = 108;
//    public static double shoot4Heading = 145;

    public static double move3X = 43;
    public static double move3Y = 72;
    public static double moveHeading = 150;




    // control points for intaking
    public static double control1X = 87.67;
    public static double control1Y = 55.5;

    public static double gate1X = 33.5;
    public static double gate1Y = 54;
    public static double gate12X = 32;
    public static double gate12Y = 74;

    public static double control2X = 87.4;
    public static double control2Y = 55.2;
    public static double control22X = 56;
    public static double control22Y = 60;


    public static double control3X = 90;
    public static double control3Y = 32;
    public static double control32X = 52.4;
    public static double control32Y = 35.3;

//    public static double control4X = 80;
//    public static double control4Y = 48;




    public static Path shoot0Path;
    public static Path intake1Path;
    public static Path gate1Path;
    public static Path shoot1Path;
    public static Path intake2Path;
    public static Path shoot2Path;
    public static Path intake3Path;
    public static Path shoot3Path;
    public static Path intake4Path;
    public static Path shoot4Path;
    public static Path movePath;


    public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    public void buildPaths() {
        Pose shoot0Pose = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0Heading));
        Pose intake1Pose = new Pose(intake1X, intake1Y, Math.toRadians(intake1Heading));
        Pose gate1Pose = new Pose(gateOpen1X, gateOpen1Y, Math.toRadians(gateOpen1Heading));
        Pose shoot1Pose = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1Heading));
        Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Heading));
        Pose shoot2Pose = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2Heading));
        Pose intake3Pose = new Pose(intake3X, intake3Y, Math.toRadians(intake3Heading));
        Pose shoot3Pose = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3Heading));
        Pose movePose = new Pose(move3X, move3Y, Math.toRadians(moveHeading));



        Pose intakeControl1 = new Pose(control1X, control1Y);

        Pose intakeControl2 = new Pose(control2X, control2Y);
        Pose intakeControl22 = new Pose(control22X, control22Y);

        Pose gateControl1 = new Pose(gate1X, gate1Y);
        Pose gateControl12 = new Pose(gate12X, gate12Y);





        Pose intakeControl3 = new Pose(control3X, control3Y);
        Pose intakeControl32 = new Pose(control32X, control32Y);





        shoot0Path = buildPath(startPose, shoot0Pose);
        intake1Path = buildCurve(shoot0Pose, intake1Pose, intakeControl1);
        gate1Path = buildCurve(intake1Pose, gate1Pose, gateControl1, gateControl12, 0.2);

        shoot1Path = buildPath(intake1Pose, shoot1Pose, 0.5);
        intake2Path = buildCurve(shoot1Pose, intake2Pose, intakeControl2, intakeControl22);
        shoot2Path = buildPath(intake2Pose, shoot2Pose, 0.5);
        intake3Path = buildCurve(shoot2Pose, intake3Pose, intakeControl3, intakeControl32);
        shoot3Path = buildPath(intake3Pose, shoot3Pose, 0.5);
//        intake4Path = buildCurve(shoot3Pose, intake4Pose, intakeControl4, 0.3);
//        shoot4Path = buildPath(intake4Pose, shoot4Pose, 0.5);
        movePath = buildPath(shoot3Pose, movePose);


    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;
        Constants.shootPower = 0.74;
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
                                new ArtifactInCommand(),
                                new ShooterStateCommand(ShooterSubsystem.ShootState.SHOOT)
                        ),

                        new WaitCommand(1000), // to let the launcher charge up


                        new WaitCommand(300),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),

                        new PathCommand(intake1Path),

                        new EverythingStopCommand(),

                        new PathCommand(gate1Path),

                        new PathCommand(shoot1Path),
                        new WaitCommand(350),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),

                        new PathCommand(intake2Path),

                        new EverythingStopCommand(),

                        new PathCommand(shoot2Path),
                        new WaitCommand(350),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),


                        new PathCommand(intake3Path),

                        new EverythingStopCommand(),

                        new PathCommand(shoot3Path),
                        new WaitCommand(350),
                        new ArtifactShootCommand(),
                        new WaitCommand(400),
                        new ArtifactLowerPowerShootCommand(),
                        new WaitCommand(800),
                        new ArtifactShootCommand(),
                        new ArtifactInCommand(),


//                        new PathCommand(intake4Path),
//
//                        new PathCommand(shoot4Path),
//                        new WaitCommand(350),
//                        new ArtifactShootCommand(),
//                        new WaitCommand(400),
//                        new ArtifactLowerPowerShootCommand(),
//                        new WaitCommand(800),
//                        new ArtifactShootCommand(),

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


//    package org.firstinspires.ftc.teamcode.opmode.auto;
//
//    import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;
//    import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
//    import com.arcrobotics.ftclib.command.CommandScheduler;
//    import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//    import com.arcrobotics.ftclib.command.WaitCommand;
//    import com.bylazar.configurables.annotations.Configurable;
//    import com.pedropathing.geometry.Pose;
//    import com.pedropathing.paths.Path;
//    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//    import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
//    import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
//    import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
//    import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
//    import org.firstinspires.ftc.teamcode.hardware.Robot;
//    import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
//    import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
//    import org.firstinspires.ftc.teamcode.util.Globals;
//
//    @Autonomous(name = "Auto_9_0")
//    @Configurable
//    public class Auto_9_0 extends LinearOpMode {
//
//
//
//        public static double startX = 20.5;
//        public static double startY = 123;
//        public static double startHeading = 135;
//
//
//        public static double shoot0X = 55;
//        public static double shoot0Y = 88;
//        public static double shoot0Heading = 135;
//
//
//        public static double intake1X = 23.5;
//        public static double intake1Y = 84;
//        public static double intake1Heading = 180;
//
//
//        public static double shoot1X = 55;
//        public static double shoot1Y = 88;
//        public static double shoot1Heading = 135;
//
//
//        public static double intake2X = 19;
//        public static double intake2Y = 60;
//        public static double intake2Heading = 180;
//
//
//        public static double shoot2X = 55;
//        public static double shoot2Y = 88;
//        public static double shoot2Heading = 135;
//
//
//        public static double intake3X = 20;
//        public static double intake3Y = 35.6;
//        public static double intake3Heading = 180;
//
//        public static double shoot3X = 55;
//        public static double shoot3Y = 88;
//        public static double shoot3Heading = 135;
//
//        public static double move3X = 70;
//        public static double move3Y = 103.5;
//        public static double moveHeading = 135;
//
//
//
//
//        // control points for intaking
//        public static double control1X = 80;
//        public static double control1Y = 84;
//
//        public static double control2X = 69;
//        public static double control2Y = 76;
//        public static double control22X = 79;
//        public static double control22Y = 56;
//
//        public static double control3X = 89.7;
//        public static double control3Y = 31.5;
//        public static double control32X = 66.5;
//        public static double control32Y = 34;
//
//
//        public static Path shoot0Path;
//        public static Path intake1Path;
//        public static Path shoot1Path;
//        public static Path intake2Path;
//        public static Path shoot2Path;
//        public static Path intake3Path;
//        public static Path shoot3Path;
//        public static Path movePath;
//
//
//        public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));
//
//        public void buildPaths() {
//            Pose shoot0Pose = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0Heading));
//            Pose intake1Pose = new Pose(intake1X, intake1Y, Math.toRadians(intake1Heading));
//            Pose shoot1Pose = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1Heading));
//            Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Heading));
//            Pose shoot2Pose = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2Heading));
//            Pose intake3Pose = new Pose(intake3X, intake3Y, Math.toRadians(intake3Heading));
//            Pose shoot3Pose = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3Heading));
//            Pose movePose = new Pose(move3X, move3Y, Math.toRadians(moveHeading));
//
//
//
//            Pose intakeControl1 = new Pose(control1X, control1Y);
//
//            Pose intakeControl2 = new Pose(control2X, control2Y);
//            Pose intakeControl22 = new Pose(control22X, control22Y);
//
//            Pose intakeControl3 = new Pose(control3X, control3Y);
//            Pose intakeControl32 = new Pose(control32X, control32Y);
//
//
//            shoot0Path = buildPath(startPose, shoot0Pose);
//            intake1Path = buildCurve(shoot0Pose, intake1Pose, intakeControl1);
//            shoot1Path = buildPath(intake1Pose, shoot1Pose);
//            intake2Path = buildCurve(shoot1Pose, intake2Pose, intakeControl2, intakeControl22);
//            shoot2Path = buildPath(intake2Pose, shoot2Pose);
//            intake3Path = buildCurve(shoot2Pose, intake3Pose, intakeControl3, intakeControl32);
//            shoot3Path = buildPath(intake3Pose, shoot3Pose);
//            movePath = buildPath(shoot3Pose, movePose);
//
//
//        }
//
//        @Override
//        public void runOpMode() {
//            Robot robot = Robot.getInstance();
//
//            Globals.IS_AUTO = true;
//
//            robot.initialize(hardwareMap, telemetry);
//            CommandScheduler.getInstance().reset();
//
//            buildPaths();
//
//            while (!isStarted() && !isStopRequested()) {
//                CommandScheduler.getInstance().run();
//                robot.updateData();
//                robot.periodic();
//                robot.write();
//            }
//
//            robot.follower.setPose(startPose);
//
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//
//                            new PathCommand(shoot0Path).alongWith(
//                                    new ArtifactInCommand()
//                            ),
//
//                            new WaitCommand(1250), // to let the launcher charge up
//
//                            new ArtifactShootCommand(),
//                            new WaitCommand(1000),
//                            new ArtifactShootCommand(),
//
//                            new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
//
//                            new PathCommand(intake1Path),
//
//                            new PathCommand(shoot1Path),
//
//                            new WaitCommand(100),
//
//                            new ArtifactShootCommand(),
//                            new WaitCommand(1000),
//                                new ArtifactShootCommand(),
//
//                                new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
//
//                                new PathCommand(intake2Path),
//
//                                new PathCommand(shoot2Path),
//
//                                new WaitCommand(100),
//
//                                new ArtifactShootCommand(),
//                                new WaitCommand(1000),
//                                new ArtifactShootCommand(),
//
//                                new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
//
//
//                                new PathCommand(intake3Path),
//
//                                new PathCommand(shoot3Path),
//
//                                new WaitCommand(100),
//
//                                new ArtifactShootCommand(),
//                                new WaitCommand(1000),
//                            new ArtifactShootCommand(),
//
//                            new PathCommand(movePath)
//                    )
//            );
//
//            while (opModeIsActive() && !isStopRequested()) {
//                CommandScheduler.getInstance().run();
//                robot.updateData();
//                robot.periodic();
//                robot.write();
//            }
//        }
//    }
