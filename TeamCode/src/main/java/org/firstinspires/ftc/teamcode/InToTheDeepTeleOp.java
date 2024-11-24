package org.firstinspires.ftc.teamcode;

/*              Driver station Configuration
Drive Motors:
leftFrontDrive                 control           motor 0
rightFrontDrive                control           motor 1
leftBackDrive                  control           motor 2
rightBackDrive                 control           motor 3

Intake Motors:
linearSlideElevator            expansion         motor 1
linearSlideARM                 expansion         motor 3
RotatingARMJoint               expansion         motor 2

Climb Motors:
climbElevator                  expansion         motor 0

Servos:
IntakeRotation                 control            servo 1
wheelSpin                      control            servo 3
hook                           control            servo 0
blinkIn                        control            servo 2                  ledDriver

Sensors
imu                            control            i2cBus 0                revInternalIMU
leftDistanceSensor             control            i2Bus 1                 leftDistance sensor
colorSensor                    control            i2Bus 2                 color sensor
rightDistanceSensor            control            i2cBus 3                rightDistance sensor
*/

/*
        Driver Station key mapping

        gamepad1.jpystick1                  drive fwd                 ||      gamepad2.joystick2  y      climbElevator up
        gamepad1.jpystick1                  drive back                ||      gamepad2.joystick2  y      climbElevator down
        gamepad1.jpystick1                  strafe left               ||      gamepad2.dpadleft          Climbhook rotation (clockwise)
        gamepad1.jpystick1                  strafe right              ||      gamepad2.dpadright         Climbhook Rotation (anticlockwise)
        gamepad1.jpystick2                  turn left                 ||      gamepad2.jpystick1 y  LB      IntakeElevator up
        gamepad1.jpystick2                  turn right                ||      gamepad2.jpystick1 y  LT     IntakeElevator down
        gamepad1.a                         initialize/reset IMU       ||      gamepad2.jpystick1 x RB     IntakeARM fwd
        gamepad1.dpadleft                  specimen high rung         ||      gamepad2.jpystick1 x RT     IntakeARM back
        gamepad1.dpadright                 specimen lower rung        ||      gamepad2.joystick2 x      RotatingARMJoint up
        gamepad1.dpadup                    samples high basket        ||      gamepad2.joystick2 x      RotatingARMJoint down
        gamepad1.dpaddown                  samples lower basket       ||      gamepad2.a                intakeRollerLefttoRight
        gamepad1.y                  Tele-Op operatorAssist            ||      gamepad2.b                intakeRollerRighttoLeft
        gamepad1.x                  all motor reset             ||      gamepad2.x                 IntakeRollersample
        gamepad1.                                                    ||      gamepad2.y                 ReleaseRollersample

    // Potential Automated Routines @ EndGame & TeleOp
    Climb Stage 2
    Climb Stage 3
    specimen low rung
    specimen high rung
    sample low basket
    sample high basket
*/

import static org.firstinspires.ftc.teamcode.RobotConstants.COLORSENSOR_DISTANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.OPERATOR_ERROR_MARGIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.OPERATOR_GAIN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_RESET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TELEOP_ASSIST_DRIVETRAIN_GAIN;
import static org.firstinspires.ftc.teamcode.RobotConstants.TELEOP_ASSIST_DRIVETRAIN_TURN_GAIN;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MaxVelocity;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FIELD_CENTRIC;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp

//Example Codes for Field Centric Drive
public class InToTheDeepTeleOp extends LinearOpMode {
    //Robot Constants for import.
    private RobotConstants constants = new RobotConstants();
    private boolean fieldConstant = FIELD_CENTRIC;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private boolean driverAssistPickup = false;


    private DcMotorEx linearSlideElevator = null;
    private DcMotorEx linearSlideARM = null;
    private DcMotorEx RotatingARMJoint;
    private CRServo IntakeDirection;
    private Servo RightClaw;
    private Servo LeftClaw;
    private DcMotorEx Climb = null;
    private CRServo hook;
    private CRServo IntakeWheelSpin;

    RevBlinkinLedDriver blinkinLedDriver;
    Rev2mDistanceSensor leftDistanceSensor;
    Rev2mDistanceSensor rightDistanceSensor;
    RevColorSensorV3 colorSensor;

    public String getSampleColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        float maxsat = Math.max(Math.max(colors.red, colors.green), colors.blue);
        float r = colors.red / maxsat;
        float g = colors.green / maxsat;
        float b = colors.blue / maxsat;

        String sample = "Nothing";

        if (distance < COLORSENSOR_DISTANCE) {
            if (r == 1.0) {
                sample = "Red";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (b == 1.00) {
                sample = "Blue";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                sample = "Yellow";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
        } else {
            //Bluegreen color - I'm chicking it to see if it can do stuff :)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
        }
        return sample;
    }

    public void initializemotor() {
        // to the names assigned during the robot co
        // Configuration step on the DS or RC devices.
        //Drive Base Init
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        // Intake Mechanism Init
        linearSlideElevator = hardwareMap.get(DcMotorEx.class, "linearSlideElevator");
        linearSlideARM = hardwareMap.get(DcMotorEx.class, "linearSlideARM");
        RotatingARMJoint = hardwareMap.get(DcMotorEx.class, "RotatingARMJoint");
        //Servos
        IntakeDirection = hardwareMap.get(CRServo.class, "IntakeDirection");
//        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
//        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        IntakeWheelSpin = hardwareMap.get(CRServo.class, "WheelSpin");

        //Ascent HW Init
        Climb = hardwareMap.get(DcMotorEx.class, "climb");
        hook = hardwareMap.get(CRServo.class, "hook");

        //Initialize the color sensor
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        //Initialize distance sensors
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

        // Configure Hardware for correct state
//      Robot Drive base direction
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        linearSlideElevator.setDirection(DcMotorEx.Direction.FORWARD);
        linearSlideARM.setDirection(DcMotorEx.Direction.FORWARD);
//        Intakerollerdirection.setPosition(0);
//        IntakeWheelSpin.setPosition(0);
        RotatingARMJoint.setDirection(DcMotorEx.Direction.FORWARD);

        Climb.setDirection(DcMotorEx.Direction.REVERSE);
//        hook.setPosition(0);

        //         Competition Robot Direction
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearSlideElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideARM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RotatingARMJoint.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Climb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        linearSlideARM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        // Elevator and shoulder rotation needs control running continuously, need to run power continuously
//        // hence set more run to position and set position and give power
//        RotatingARMJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
////        RotatingARMJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updatedrivebase(double lf, double lb, double rf, double rb) {
        // Send calculated power to wheels
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    public void updatetelemetry_26295(double heading) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getVelocity(), rightFrontDrive.getVelocity());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getVelocity(), rightBackDrive.getVelocity());
//            telemetry.addData("non-Calibrated  Axial/Lateral", "%4.2f, %4.2f", axial, lateral);
//            telemetry.addData("Calibrated  Axial/Lateral", "%4.2f, %4.2f", Adjaxial, Adjlateral);
        telemetry.addData("heading ", "%4.2f", heading);

        telemetry.addData("Linear Slide Elevator", linearSlideElevator.getCurrentPosition());
        telemetry.addData("Linear SlideARM ", linearSlideARM.getCurrentPosition());
        telemetry.addData("rotatingARM", RotatingARMJoint.getCurrentPosition());

        telemetry.addData("IntakeWheel Power ", IntakeDirection.getPower());

        telemetry.addData("climb ", Climb.getCurrentPosition());
//        telemetry.addData("Sample detected", getSampleColor());
        telemetry.addData("Left distance", leftDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Front distance", rightDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("WheelIntake", IntakeWheelSpin.getPower());
        telemetry.update();

    }

    public void runOpMode() throws InterruptedException {


        // Initialize the hardware variables. Note that the strings used here must correspond
        initializemotor();

        //Initialize IMU
        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //

        IMU imu = hardwareMap.get(IMU.class, "imu");
/*      Test Robot Directions
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
*/
        // Competition Robot Directions
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
//      IMU calibration
        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & rotate, and right joystick to strafe.
            double axial = -gamepad1.left_stick_y * TELEOP_ASSIST_DRIVETRAIN_GAIN;  //FWD
            double lateral = gamepad1.left_stick_x * TELEOP_ASSIST_DRIVETRAIN_GAIN;  //TUR
            //Test This code to check drive and turn operation.
            double yaw = gamepad1.right_stick_x * TELEOP_ASSIST_DRIVETRAIN_TURN_GAIN; //STR
            //timeout happens then reset
            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            //   Field Oriented Conversion Test
            // This is the built-in IMU in the REV hub.
            // We're initializing it by its default parameters
            // and name in the config ('imu'). The orientation
            // of the hub is important. Below is a model
            // of the REV Hub and the orientation axes for the IMU.
            //
            //                           | Z axis
            //                           |
            //     (Motor Port Side)     |   / X axis
            //                       ____|__/____
            //          Y axis     / *   | /    /|   (IO Side)
            //          _________ /______|/    //      I2C
            //                   /___________ //     Digital
            //                  |____________|/      Analog
            //
            //                 (Servo Port Side)
            //
            double Adjaxial = axial;
            double Adjlateral = lateral;
            double heading =0;
            if(fieldConstant) {
                 heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                 Adjlateral = -axial * Math.sin(heading) + lateral * Math.cos(heading);
                 Adjaxial = axial * Math.cos(heading) + lateral * Math.sin(heading);
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = Adjaxial + Adjlateral + yaw;
            double rightFrontPower = Adjaxial - Adjlateral - yaw;
            double leftBackPower = Adjaxial - Adjlateral + yaw;
            double rightBackPower = Adjaxial + Adjlateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Read joystick for DC motor operation.
            double climbVar = gamepad2.right_stick_y;
            double ARMjointVar = gamepad2.right_stick_x;
            double elevatorVar = gamepad2.left_stick_y;
            double ARMVar = gamepad2.left_stick_x;
//          minimize opertor error with accidental angle push
            if ((climbVar > -OPERATOR_ERROR_MARGIN) && (climbVar < OPERATOR_ERROR_MARGIN))
                climbVar =0;
            if ((ARMjointVar > -OPERATOR_ERROR_MARGIN) && (ARMjointVar < OPERATOR_ERROR_MARGIN))
                ARMjointVar =0;
            if ((elevatorVar > -OPERATOR_ERROR_MARGIN) && (elevatorVar < OPERATOR_ERROR_MARGIN))
                elevatorVar =0;
            if ((ARMVar > -OPERATOR_ERROR_MARGIN) && (ARMVar < OPERATOR_ERROR_MARGIN))
                ARMVar=0;

            //Manual Operation for ARM, sholder, Elevator, Climb, Hook
            //          Climber Logic
//             Below this is code to get the arm and climb working
            if(climbVar != 0)
             {
                Climb.setPower(climbVar);
                climbVar=0;
            }
            else {
                Climb.setPower(0);
            }

            // Move ARM Joint (Shoulder) to desired Position.
            if ((ARMjointVar != 0)
            ) {
                //Ensure that downward drop is controlled motion with position controlled movement.
                if(ARMjointVar < 0) {
                    RotatingARMJoint.setTargetPosition(ROTATING_ARM_JOINT_RESET_POSITION);
                    RotatingARMJoint.setPower(1);
                }
                RotatingARMJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                RotatingARMJoint.setPower(ARMjointVar);
                // keep running motor to control loop
                ARMjointVar=0;
            }
            else {
                RotatingARMJoint.setTargetPosition(RotatingARMJoint.getCurrentPosition());
                RotatingARMJoint.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                RotatingARMJoint.setPower(0);
//               RotatingARMJoint.setTargetPosition(ROTATING_ARM_JOINT_RESET_POSITION);
            }
            if ((elevatorVar != 0)
            ) {
                // Move Elevator to desired Position.
//                    linearSlideElevator.setTargetPosition(ELEVATOR_HIGH_BASKET_POSITION);
//                linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                linearSlideElevator.setPower(elevatorVar);
                elevatorVar=0;
            }
            else {
                linearSlideElevator.setPower(0);
////                    linearSlideElevator.setTargetPosition(ELEVATOR_RESET_POSITION);
//                linearSlideElevator.setTargetPosition(linearSlideElevator.getCurrentPosition());
////                linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                linearSlideElevator.setTargetPosition(linearSlideElevator.getCurrentPosition());
                //                    linearSlideElevator.setPower(0);
            }
            if (ARMVar != 0) {
                linearSlideARM.setPower(ARMVar* OPERATOR_GAIN_MULTIPLIER);
                ARMVar=0;
            }
            else {
//                    linearSlideARM.setPower(0);
            }
            if (gamepad2.dpad_up) {
                hook.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                hook.setPower(-1);
            }
            else {
                hook.setPower(0);
            }
//                // Wheel Spine
                if (gamepad2.a) {
                    IntakeWheelSpin.setPower(+1.0);
//                    IntakeWheelSpin.setDirection(CRServo.Direction.FORWARD);
                }
                else if(gamepad2.b) {
                    IntakeWheelSpin.setPower(-1.0);
//                    IntakeWheelSpin.setDirection(CRServo.Direction.REVERSE);
                }
                else {
                    IntakeWheelSpin.setPower(0);
                }
//            if (gamepad2.a) {
//                RightClaw.setPosition(1.0);
//                LeftClaw.setPosition(1.0);
////                    IntakeWheelSpin.setDirection(CRServo.Direction.FORWARD);
//            }
//            else if(gamepad2.b) {
////                RightClaw.setPosition(1.0);
////                LeftClaw.setPosition(1.0);
//                    IntakeWheelSpin.setDirection(CRServo.Direction.REVERSE);
//            }
                // Intakerollerdirection
                if(gamepad2.y) {
                    IntakeDirection.setPower(-0.25);
                }
                else if (gamepad2.x) {
                    IntakeDirection.setPower(0.25);
                }
                else {
                    IntakeDirection.setPower(0);
                }
                if (gamepad2.dpad_left) {
                    linearSlideARM.setPower(1);
                }
                else if (gamepad2.dpad_right) {
                    linearSlideARM.setPower(-1);
                }
                else {
                    linearSlideARM.setPower(0);
                }

            // Send calculated power to wheels double lf, double lb, double rf, double rb
            updatedrivebase(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

            // Show the elapsed game time and wheel power.c
            updatetelemetry_26295(heading);
        }
    }
}