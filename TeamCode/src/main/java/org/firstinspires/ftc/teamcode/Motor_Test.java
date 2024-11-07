package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.hardware.RevIMU;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//Example Codes for Robot Centric Drive
//use this function to test motors and calibrate DC motors and Servo
public class Motor_Test extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx linearSlideElevator = null;
    private DcMotorEx linearSlideARM = null;
    private DcMotorEx RotatingARMJoint;
    private CRServo Intakerollerdirection;
    private CRServo IntakeWheelSpin;

    private DcMotorEx Climb = null;
    private CRServo hook;

    RevBlinkinLedDriver blinkinLedDriver;
    Rev2mDistanceSensor leftDistanceSensor;
    Rev2mDistanceSensor rightDistanceSensor;

    public void updatetelemetry_26295(double heading) {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
//            telemetry.addData("non-Calibrated  Axial/Lateral", "%4.2f, %4.2f", axial, lateral);
//            telemetry.addData("Calibrated  Axial/Lateral", "%4.2f, %4.2f", Adjaxial, Adjlateral);
        telemetry.addData("heading ", "%4.2f", heading);

        telemetry.addData("Linear Slide Elevator", linearSlideElevator.getCurrentPosition());
        telemetry.addData("Linear SlideARM ", linearSlideARM.getCurrentPosition());
        telemetry.addData("rotatingARM", RotatingARMJoint.getCurrentPosition());

        telemetry.addData("IntakeWheel Power ", Intakerollerdirection.getPower());

        telemetry.addData("climb ", Climb.getCurrentPosition());
//        telemetry.addData("Sample detected", getSampleColor());
        //       telemetry.addData("Left distance", leftDistanceSensor.getDistance(DistanceUnit.MM));
        //     telemetry.addData("Front distance", rightDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("WheelIntake", IntakeWheelSpin.getPower());
        telemetry.update();

    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        // Intake Mechanism Init
        linearSlideElevator = hardwareMap.get(DcMotorEx.class, "linearSlideElevator");
        linearSlideARM = hardwareMap.get(DcMotorEx.class, "linearSlideARM");
        RotatingARMJoint = hardwareMap.get(DcMotorEx.class, "RotatingARMJoint");
        //Servos
        Intakerollerdirection = hardwareMap.get(CRServo.class, "IntakeRotation");
        IntakeWheelSpin = hardwareMap.get(CRServo.class, "wheelSpin");
        //Ascent HW Init
        Climb = hardwareMap.get(DcMotorEx.class, "climb");
        hook = hardwareMap.get(CRServo.class, "hook");


        //Initialize distance sensors
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");


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
//      Test Robot Drive base direction
/*        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
*///        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//          Robot Directions

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

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideARM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RotatingARMJoint.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Climb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

       leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideARM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Elevator and shoulder rotation needs control running continuously, need to run power continuously
        // hence set more run to position and set position and give power
        RotatingARMJoint.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        RotatingARMJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       //  Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        updatetelemetry_26295(1.0);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & rotate, and right joystick to strafe.
            double axial   = -gamepad1.left_stick_y;  //fwd
            double lateral =  gamepad1.left_stick_x;  //TUR
            double yaw     =  gamepad1.right_stick_x; //STR

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            double climbVar = gamepad1.dpad_right? 1.0: 0.0;
            double ARMjointVar = gamepad1.dpad_down ? 1.0: 0.0;
            double elevatorVar = gamepad1.dpad_up ? 1.0: 0.0;
            double ARMVar = gamepad1.dpad_left ? 1.0: 0.0;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            RotatingARMJoint.setPower(ARMjointVar);
            linearSlideElevator.setPower(elevatorVar);
            linearSlideARM.setPower(ARMVar);
            Climb.setPower(climbVar);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            updatetelemetry_26295(1.0);
        }
    }

}