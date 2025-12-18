package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    //Tolerance for paths
    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);

    //Robot Centric is false, allows for field centric to be active
    public static boolean RobotCentric = false;

    //Intake Constants
    public static double intakeInPower = -1;
    public static double intakeOutPower = 1;

    //Shoot Constants
    public static double shootPower = 0.7;
    public static double readyPower = -1;
    public static double readySlowPower = -0.5;
    public static double reverseStopPower = 1;

    //Servo Constants
    public static double blockerBlock = 0.5;
    public static double blockerOpen = 0.99;

    //Delays
    public static double shootDelay = 130;
    // I might make it lower depending on how fast we want the shots to be

    // PIDF constants
    public static double kP = 0.0004; // to make response faster, Proportional; Responds to the current error.
    //Example: If your shooter wheel is spinning too slowly, proportional control increases motor power proportionally to how far off it is
    public static double kI = 0.00005; // for undershoot, Integral; Responds to the accumulated error over time.
    //Example: If your wheel is always 50 RPM below target, integral builds up and adds extra correction until it matches.
    public static double kD = 0.0; // Don't Change, Derivative; Responds to the rate of change of error.
    //Example: If the wheel is accelerating too quickly toward the target, derivative reduces power to avoid overshooting.
    public static double kF = 32767 /((1440 * 6000) / 60.0);// Default need to change, -  Feedforward; Predicts the needed output based on the target setpoint, before any error occurs.
    //- Example: If you know your shooter needs 60% motor power to hold 3000 RPM, feedforward applies that baseline immediately, and PID fine‑tunes around it.

    //Turret Constants
    //Controller Helper Params
    public static double deadBandDeg = 0.3;// This is the amount of small range of error where the controller does nothing
    public static double errAlpha = 0.35;// This is the smoothening factor, in this case with 35 it gives 35 percent weight to new data and 65 percent to old data, this prevents jitters and prevents teh derivatives from blowing up.
    // Safety Rails
    public static double maxIntegral = 30.0;//It is the integral term, accumulates error over time.
    public static double maxDeriv = 320; // It is the maximium rate of error/change the controller can face

    //CR Servo output limits
    public static double maxPower = 1.0;
    public static double kS = 0.0;//set to 0.03 is something is still messing up
    // It offsets the minimum power needed to overcome servo deadband or friction

    //PID gain mapping error -> power (Need to tune these)
    public static double kP_v = 0.020;//This is the same as the PIDF constants above except that it is for the turret
    public static double kI_v = 0.000;
    public static double kD_v = 0.0010;

}