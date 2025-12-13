package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.EverythingStopCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

import java.util.function.Supplier;

@Disabled
//@TeleOp (name = "SoloRealLowPower")
public class TeleOp_SoloSlow extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private GamepadEx g1;

    public static Pose startingPose;
    private TelemetryManager telemetryM;
    private Supplier<PathChain> goShootPath;
    private boolean automatedDrive;


    boolean teleOpEnabled = false;

    boolean grabConfirmed = false;

    double fieldCentricOffset;

    boolean lastLeftTrigger;
    boolean lastRightTrigger;

    boolean lastA;
    boolean lastB;
    boolean lastX;
    boolean lastY;

    boolean lastLeftBumper;
    boolean lastRightBumper;

    boolean lastDpadUp;
    boolean lastDpadDown;
    boolean lastDpadLeft;
    boolean lastDpadRight;

    boolean lastRightStickButton;
    boolean lastLeftStickbutton;

    boolean lastPS;
    boolean lastStart;
    boolean lastBack;


    @Override
    public void initialize() {

        Constants.shootPower = 0.73;

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        goShootPath = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();

        robot.initialize(hardwareMap, telemetry);

    }

    @Override
    public void run() {

        if (teleOpEnabled) {

            CommandScheduler.getInstance().run();

            robot.periodic();
            robot.updateData();
            robot.write();

            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x/2,
                    Constants.robotCentric // Robot Centric
            );

        }


        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean y = g1.getButton(GamepadKeys.Button.Y);
        boolean leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean dpadUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean dpadDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadLeft = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean rightStickButton = g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean leftStickButton = g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        boolean ps = gamepad1.ps;
        boolean start = g1.getButton(GamepadKeys.Button.START);
        boolean back = g1.getButton(GamepadKeys.Button.BACK);



        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);
            if (Constants.robotCentric) gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }



        if (a && !lastA) {
            CommandScheduler.getInstance().schedule(new ShooterStateCommand(ShooterSubsystem.ShootState.SHOOT));
        }

        if (b && !lastB) {
            CommandScheduler.getInstance().schedule(new ShooterStateCommand(ShooterSubsystem.ShootState.STOP));
        }

        if (leftBumper && !lastLeftBumper) {
            CommandScheduler.getInstance().schedule((new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)));
        }

        if(rightBumper && !rightBumper) {
            CommandScheduler.getInstance().schedule(new EverythingStopCommand());
        }


        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastRightStickButton = rightStickButton;
        lastLeftStickbutton = leftStickButton;
        lastPS = ps;
        lastStart = start;

        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (rightTrigger && !lastRightTrigger) {
//            robot.follower.followPath(goShootPath.get()); // remove if needed
//            automatedDrive = false;
            CommandScheduler.getInstance().schedule(new ArtifactShootCommand());
        }

        if (leftTrigger && !lastLeftTrigger) {
//            robot.follower.startTeleopDrive();
            CommandScheduler.getInstance().schedule(new ArtifactInCommand());

        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;


        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose());
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);

        }


    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}
