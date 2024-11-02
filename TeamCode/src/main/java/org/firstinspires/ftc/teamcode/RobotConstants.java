package org.firstinspires.ftc.teamcode;

public class RobotConstants
{

    public static final double OPERATOR_ERROR_MARGIN = 0.5;//wrounding error on joysticlk
    public static final double OPERATOR_MULTIPLIER = 1;//50 % power
    public static final double COLORSENSOR_DISTANCE = 5;//5 cm

    public static int ELEVATOR_HIGH_LIMIT_POSITION = -17484;
    public static int ELEVATOR_HIGH_BASKET_POSITION = -5508;// position for shoulder for basket drop
    public static int ELEVATOR_HIGH_SPECIMEN_HANG_POSITION = -5508;// position for shoulder for basket drop
    public static int ELEVATOR_RESET_POSITION =-6; // start position

    public static int ROTATING_ARM_JOINT_HIGH_POSITION = 520;   // vertical position
    public static int ROTATING_ARM_JOINT_BASKET_POSITION = 278;// position for shoulder for basket drop
    public static int ROTATING_ARM_JOINT_SPECIMEN_HANG_POSITION = 278;// position for shoulder for basket drop
    public static int ROTATING_ARM_JOINT_RESET_POSITION =0; // start position

    public static int LINEAR_ARM_RESET_POSITION =-126; // start position
    public static int LINEAR_ARM_BASKET_POSITION = 1176; // start position

    //adjust this value to ensure smooth drive operation
    public static int TELEOP_ASSIST_DRIVETRAIN_GAIN = 1; // % multiplier for drive smooth operation
    public static int TELEOP_ASSIST_DRIVETRAIN_TURN_GAIN = (int) 0.5; // % gain multiplier for smooth turning operation - slower is better

    //Specimen assist function
    public static int HIGH_RUNG_SPECIMEN_ROBOT_FRONT_POSITION =0; // distance sensor

    //Upper basket assist function
    public static int UPPER_BASKET_SAMPLE_ROBOT_FRONT_POSITION =0; // distance sensor
    public static int UPPER_BASKET_SAMPLE_ROBOT_SIDE_POSITION =0; // distance sensor

//    public static final double DRIVEBASE_MULTIPLIER = 1.0;//-1.2;//Bot A = 1.0, Bot B = -1.0
//    //Intake Mechanism
//    public static final int LINEARSLIDEELEVATOR_RESET_POSITION = -7210;//Lowest position, on the floor
//    public static final int LINEARSLIDEELEVATOR_TOP_RUNG_PLACE = 3330;//Upper rung starting position
//    public static final int LINEARSLIDEELEVATOR_LOWER_RUNG_PLACE = 0;//Upper rung starting position
//    public static final int LINEARSLIDEELEVATOR_TOP_BASKET_PLACE = -2100;//Upper basket starting position
//    public static final int LINEARSLIDEELEVATOR_LOWER_BASKET_PLACE = 0;//Upper basket starting position
//    public static final int LINEARSLIDEARM_RESET_POSITION = 0;//Upper rung starting position
//    public static final int LINEARSLIDEARM_UPPER_BASKET_POSITION = 0;//Upper basket sample position
//    public static final int LINEARSLIDEARM_LOWER_BASKET_POSITION = 0;//lower basket sample position
//    public static final int LINEARSLIDEARM_UPPER_RUNG_POSITION = 0;//Upper RUNG specimen position
//    public static final int LINEARSLIDEARM_LOWER_RUNG_POSITION = 0;//lower RUNG specimen position
//    public static final int ARMJOINT_RESET_POSITION = 0;//lower basket sample position
//    public static final int ARMJOINT_UPPER_POSITION = 0;//Max limit for rotating ARM
//    public static final int ARMJOINT_LOWER_POSITION = 0;//Low position limit for Rotating ARM
//    // Climb mechanism
//    public static final int CLIMBELEVATOR_RESET_RELEASE = 0;//Reset Position
//    public static final int CLIMBELEVATOR_TOP_RUNG_RELEASE = 0;//Upper rung pull down position
//    public static final int CLIMBELEVATOR_LOWER_RUNG_RELEASE = -1450;//Upper rung pull down position
//    public static final int CLIMBHOOK_RESET_POSITION= 0;//Upper rung pull down position
//    public static final int CLIMBHOOK_TOP_RUNG_POSITION = 0;//Upper rung pull down position
//    public static final int CLIMBHOOK_LOWER_RUNG_POSITION = -1450;//Upper rung pull down position

//  pickup
    public static final int DISTANCE_SPECIMEN_PICKUP = -725;
    /*
    Start:
        Rotating Arm Start constant; 0
        Linear Slide elevator : -6
        Drive motors ; Unimportant
        Linear SLide Arm -126

    Lower Basket;
        Rotiaing arm : 278
        Linear Slide Arm: 1176
        Right Disatnce: 540
        Left Diatance: 540
        INtake Rotoation: NTFO
        Linaer Slide elevator : -6

    Upper Basket:
        Linear slide elevatrot: -5508
        ALL OTHER PARAMETERS ARE THE SAME
     */






}
