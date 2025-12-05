package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BlockerStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@Disabled
@TeleOp(name = "SoloReal")
public class TeleOp_Solo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx g1;

    public static Pose startingPose;

    private boolean teleOpEnabled = false;

    // Button state tracking
    private boolean lastLeftTrigger;
    private boolean lastRightTrigger;

    private boolean lastA;
    private boolean lastB;
    private boolean lastX;
    private boolean lastY;

    private boolean lastLeftBumper;
    private boolean lastRightBumper;

    private boolean lastDpadUp;
    private boolean lastDpadDown;
    private boolean lastDpadLeft;
    private boolean lastDpadRight;

    private boolean lastRightStickButton;
    private boolean lastLeftStickbutton;

    private boolean lastPS;
    private boolean lastStart;
    private boolean lastBack;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

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
                    -gamepad1.right_stick_x / 2.0,
                    Constants.robotCentric // Robot Centric
            );
        }

        // Read buttons
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

        // Robot-centric toggle on X rising edge (no command scheduling needed)
        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);

            if (Constants.robotCentric) {
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }

        // Enable teleOp on START rising edge (no command scheduling needed)
        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }

        // --- Use helper for all command scheduling ---

        // A: SHOOT
        scheduleCommand(
                lastA,
                a,
                new ShooterStateCommand(ShooterSubsystem.ShootState.SHOOT)
        );

        // B: STOP SHOOTER
        scheduleCommand(
                lastB,
                b,
                new ShooterStateCommand(ShooterSubsystem.ShootState.STOP)
        );

        // Left bumper: stop intake, wait, then open blocker
        scheduleCommand(
                lastLeftBumper,
                leftBumper,
                new SequentialCommandGroup(
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP),
                        new WaitCommand(150),
                        new BlockerStateCommand(ShooterSubsystem.BlockerState.OPEN)
                )
        );

        // Right bumper: intake OUT
        scheduleCommand(
                lastRightBumper,
                rightBumper,
                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT)
        );

        // Update last button states (digital buttons)
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
        lastBack = back;

        // Triggers as digital buttons
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;

        // Right trigger: artifact shoot command
        scheduleCommand(
                lastRightTrigger,
                rightTrigger,
                new ArtifactShootCommand()
        );

        // Left trigger: intake IN, reverse stop wheel, block
        scheduleCommand(
                lastLeftTrigger,
                leftTrigger,
                new SequentialCommandGroup(
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                        new StopStateCommand(ShooterSubsystem.StopState.REVERSE),
                        new BlockerStateCommand(ShooterSubsystem.BlockerState.BLOCKING)
                )
        );

        // Update last trigger states
        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;

        // Touchpad: reset pose (no command scheduling needed)
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