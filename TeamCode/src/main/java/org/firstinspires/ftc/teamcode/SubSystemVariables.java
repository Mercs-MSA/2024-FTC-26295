package org.firstinspires.ftc.teamcode;


//@Config
public class SubSystemVariables {

    public static int currentBot = 0;//0 = New robot, 1 = Original robot

    public static boolean protectEndgame = false;
    public static int headingToBackboard;

    public static final int INTAKE_LIFT_POS_0 = 143;
    public static final int INTAKE_LIFT_POS_1 = 5;
    public static final int INTAKE_LIFT_POS_2 = 0;
    public static final int INTAKE_LIFT_POS_3 = 800;
    public static final double INTAKE_INTAKE_SPEED = 1;
    public static final double INTAKE_OUTTAKE_SPEED = -.6;

    public static final double CLAW_ARM_POWER_AUTO = 0.3;
    public static final double CLAW_ARM_POWER = 0.6;
    public static final int CLAW_ARM_POS_0 = 570;
    public static final int CLAW_ARM_POS_1 = 5;
    public static final int CLAW_ARM_POS_2 = 250;
    public static final int CLAW_ARM_POS_3 = 250;


    public static final double INTAKE_LIFT_POWER = 0.4;

    public static final double HANG_LIFT_HANG_POWER = 0.6;
    public static final double HANG_LIFT_DROP_POWER = 0.2;
    public static boolean CLAW_OPEN = false;
    public static double STRAFE_SPEED = 0.7;

    public static enum ALLIANCE_COLOR {BLUE, RED};
    public static enum ALLIANCE_SIDE {BOTTOM, TOP};
    public static ALLIANCE_COLOR allianceColor = ALLIANCE_COLOR.BLUE;
    public static ALLIANCE_SIDE allianceSide = ALLIANCE_SIDE.TOP;
    public static int parkingPos = 1;
    public static boolean parkInBackstage = true;

    public static double HOPPER_GATE_OPEN = 0.8;
    public static double HOPPER_GATE_CLOSE = 1;

    public static double HOPPER_LIFT_POWER = 1;
    public static int HOPPER_LIFT_POS_DOWN = 0;
    public static int HOPPER_LIFT_POS_1 = -1000;
    public static int HOPPER_LIFT_POS_DELTA = -600;

    public static int HOPPER_LIFT_POS_MAX = -4200;
    public static int HOPPER_LIFT_POS_MIN = 0;

    public static double HOPPER_POS_1 = 1.0;
    public static double HOPPER_POS_2 = 0.8;
    public static double HOPPER_POS_3 = 0.7;
    public static double HOPPER_POS_4 = 0.6;
    public static double HOPPER_POS_UP = 0.7;
    public static double HOPPER_POS_DOWN = 1.0;

    public static int AUTON_TOP_BACKBOARD_DISTANCE = 38;
    public static int AUTON_BOTTOM_BACKBOARD_DISTANCE = 86;

    public static double droneLaunchVal = 0.41;

    public static double droneNoLaunchVal = 0.71;

}
