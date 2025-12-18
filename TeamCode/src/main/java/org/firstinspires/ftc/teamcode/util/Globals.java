package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Globals {
    // Define what hardware map is
    public static HardwareMap hardwareMap;
    //Define and assign true to LIMITS
    public static boolean LIMITS = true;
    //Define and assign false to IS_AUTO, this is for conditionals in AUTO to do things.
    public static boolean IS_AUTO = false;
    //Define and assign BLUE from my enum of colors to ALLIANCE
    public static COLORS ALLIANCE = COLORS.BLUE;

    public enum COLORS{
        RED,
        BLUE,
        GREEN,
        PURPLE,
        NONE
    }
}
