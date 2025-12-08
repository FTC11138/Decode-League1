package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;


public class ShooterSubsystem extends RE_SubsystemBase {


    private final DcMotorEx shootMotor;


    private final DcMotorEx stopMotor;

    private final Servo blockerServo;


    public enum ShootState {
        LOWERPOWER,
        SHOOT,
        STOP
    }
    public enum StopState {
        READY,
        READYSLOW,
        STOP,
        REVERSE
    }
    public enum BlockerState {
        BLOCKING,
        OPEN
    }

    public ShootState shootState;
    public StopState stopState;
    public BlockerState blockerState;


    private PIDController shooterPID = new PIDController(0, 0, 0);

    private static double tickPerRev = 28;
    private static final double maxRpm = 6000.0;
    private static final double maxTicksPerSecond = (tickPerRev * maxRpm) / 60.0;


    private double targetVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, String shootroller, String stoproller, String blockerservo) {
        shootMotor = hardwareMap.get(DcMotorEx.class, shootroller);
        stopMotor = hardwareMap.get(DcMotorEx.class, stoproller);
        blockerServo = hardwareMap.get(Servo.class, blockerservo);


        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF));

        tickPerRev = shootMotor.getMotorType().getTicksPerRev();

        shooterPID.setPID(Constants.kP, Constants.kI, Constants.kD);


        shootState = ShootState.STOP;
        stopState = StopState.STOP;
        blockerState = BlockerState.BLOCKING;

        // Register subsystem in Robot container
        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.shootState = shootState;
        Robot.getInstance().data.stopState = stopState;

        Robot.getInstance().data.shootVelocity = shootMotor.getVelocity();
        Robot.getInstance().data.shootTargetVelocity = targetVelocity;
    }

    public void updateShootState(ShootState newState) {
        shootState = newState;
    }

    public void updateStopState(StopState newState) {
        stopState = newState;
    }

    public void updateBlockerState(BlockerState newState) {
        blockerState = newState;
    }

    @Override
    public void periodic() {

        shooterPID.setPID(Constants.kP, Constants.kI, Constants.kD);

        switch (shootState) {
            case LOWERPOWER:
                shootMotor.setPower(0.5);
//                targetVelocity = 0.75 * maxTicksPerSecond;
                break;

            case SHOOT:

//                shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                shootMotor.setPower(Constants.shootPower);
//                targetVelocity = Constants.shootPower * maxTicksPerSecond;
                break;

            case STOP:
                shootMotor.setPower(0);
                targetVelocity = 0;
                break;
        }


        switch (stopState) {
            case READY:
                stopMotor.setPower(Constants.readyPower);
                break;
            case READYSLOW:
                stopMotor.setPower(Constants.readySlowPower);
                break;
            case STOP:
                stopMotor.setPower(0);
                break;
            case REVERSE:
                stopMotor.setPower(Constants.reverseStopPower);
                break;
        }

        switch (blockerState) {
            case BLOCKING:
                blockerServo.setPosition(Constants.blockerBlock);
                break;
            case OPEN:
                blockerServo.setPosition(Constants.blockerOpen);
                break;
        }
    }



    public double getCurrentVelocity() {
        return shootMotor.getVelocity();
    }



    public double getCurrentRPM() {
        return (shootMotor.getVelocity() / tickPerRev) * 60.0;
    }



    public double getTargetVelocity() {
        return targetVelocity;
    }



    public void setTargetVelocity(double velocityTicksPerSecond) {
        targetVelocity = velocityTicksPerSecond;
        shootMotor.setVelocity(targetVelocity);
    }


    public void stopShooter() {
        targetVelocity = 0;
        shootMotor.setPower(0);
    }
}


