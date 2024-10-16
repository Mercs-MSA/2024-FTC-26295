package org.firstinspires.ftc.teamcode;
// Gamepad mapping GAME pad1
//  VAR                         Location           Port                       Description
// leftFrontDrive               Control Hub        Motor 0                frontLeft Drive motor
// rightFrontDrive              Control Hub        Motor 1               frontRight Drive motor
// leftBackDrive                Control Hub        Motor 2               backLeft Drive motor
// rightBackDrive               Control Hub        Motor 3               backRight Drive motor
// intakeRollerWheel            control            servo           ROller intake driveIntakeServo servo
// intakeDirection              control Hub        servo                direction control for servo
// hook                         control Hub        servo                Hook for Climber - Ascent stage 2/3
// blinkIn                      control            servo                  ledDriver

// linearSlideElevator        Expansion Hub          motor 0                 elevator motor
// linearSlideForearm         Expansion Hub          motor 1                forram expansion motor
// rotatingElbow              Expansion Hub          motor 2                Rotating ARM joint
// Climb                      Expansion Hub          motor 3                Climber

// imu                       control            i2cBus 0                revInternalIMU
// frontrightDistanceSensor        control            i2Bus 1                 leftDistance sensor
// colorSensor               control            i2Bus 2                 color sensor
// frontleftDistanceSensor       control            i2cBus 3                frontDistance sensor


//import static com.sun.tools.javac.tree.JCTree.Tag.AND;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp

//Example Codes for Field Centric Drive
public class InToTheDeepTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = true;// CONTROL HUB MUST BE ON THE ROBOT

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx linearSlideElevator = null;
    private DcMotorEx linearSlideForearm = null;
    private DcMotorEx rotatingElbow;
    private Servo intakeDirection;
    private Servo intakeRollerWheel;

    private DcMotorEx Climb = null;
    private Servo hook;

//    private ColorSensor colorSensor;
//    private DistanceSensor frontrightDistanceSensor;
//    private DistanceSensor frontleftDistanceSensor;

    NormalizedColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    Rev2mDistanceSensor frontrightDistanceSensor;
    Rev2mDistanceSensor frontleftDistanceSensor;
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot co
        // Configuration step on the DS or RC devices.
        //Drive Base Init
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        // Intake Mechanism Init
        linearSlideElevator = hardwareMap.get(DcMotorEx.class, "linearSlideElevator");
        linearSlideForearm = hardwareMap.get(DcMotorEx.class, "linearSlideForearm");
        rotatingElbow = hardwareMap.get(DcMotorEx. class, "RotatingElbow");
        //Servos
        intakeDirection = hardwareMap.get(Servo.class, "intakeDirection");
        intakeRollerWheel =  hardwareMap.get(Servo.class, "wheelSpin");
        //Ascent HW Init
        Climb = hardwareMap.get(DcMotorEx.class, "climb");
        hook = hardwareMap.get(Servo.class, "hook");

        //Initialize the color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //Initialize distance sensors
        frontleftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontleftDistanceSensor");
        frontrightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontrightDistanceSensor");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

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


        IMU imu = hardwareMap.get(IMU.class,"imu");
/*      Test Robot Directions
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
*/
/*        // Competition Robot Directions
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
 */
        // Test Robot Directions
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize (parameters);
//      IMU calibration
        Deadline gamepadRateLimit = new Deadline(500,TimeUnit.MILLISECONDS);
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
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        linearSlideElevator.setDirection(DcMotor.Direction.FORWARD);
        linearSlideForearm.setDirection(DcMotor.Direction.FORWARD);
        intakeDirection.setPosition(0);
        intakeRollerWheel.setPosition(0);
        rotatingElbow.setDirection(DcMotor.Direction.FORWARD);

        Climb.setDirection(DcMotor.Direction.FORWARD);
        hook.setPosition(0);
//         Competition Robot Direction


        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearSlideElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideForearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotatingElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotatingElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideForearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & rotate, and right joystick to strafe.
            double axial   =  -gamepad1.left_stick_y;  //FWD
            double lateral =  gamepad1.left_stick_x;  //TUR
            double yaw     =  gamepad1.right_stick_x; //STR
            //timeout happens then reset
            if(gamepadRateLimit.hasExpired() && gamepad1.a){
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

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double Adjlateral = -axial *Math.sin(heading) + lateral * Math.cos(heading);
            double Adjaxial = axial *Math.cos(heading) + lateral * Math.sin(heading);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = Adjaxial + Adjlateral + yaw;
            double rightFrontPower = Adjaxial - Adjlateral - yaw;
            double leftBackPower   = Adjaxial - Adjlateral + yaw;
            double rightBackPower  = Adjaxial + Adjlateral - yaw;

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
            // Climber Logic
//             Below this is code to get the arm and climb working
            if ( gamepad2.dpad_up == true){
                Climb.setPower(1.0);

            }
            else if (gamepad2.dpad_down == true) {
                Climb.setPower(-1.0);
            }
            else {
                Climb.setPower(0.0); // remember to turn off if nothing pressed!
            }            //hook
            if (gamepad2.dpad_left){
                hook.setPosition(1);
            }
            else if (gamepad2.dpad_right){
                hook.setPosition(-1);
            }

            // Wheel SPin
           if (gamepad2.x == true){
               intakeRollerWheel.setPosition(1);
           } else if (gamepad2.b == true) {
               intakeRollerWheel.setPosition(-1);
           }
           else {
               intakeRollerWheel.setPosition(0);
           }

            // intakeDirection -
            if (gamepad2.right_bumper == true){
                intakeDirection.setPosition(1.0);

            }

             else if (gamepad2.left_bumper == true) {
                intakeDirection.setPosition(-1.0);
            }
            else {
                intakeDirection.setPosition(0.0);
            }
// Linear Slide Intake Up and Down
                if ((gamepad2.x == true) &&  (linearSlideElevator.getCurrentPosition() > -7210)) {
                    linearSlideElevator.setPower(0.5);

                } else if ((gamepad2.y == true) && (linearSlideElevator.getCurrentPosition() < 3330)){
                    linearSlideElevator.setPower(-1.0);
                } else {
                    linearSlideElevator.setPower(0.0);
                }
// Linear Slide Left Right\
                if (gamepad2.a == true) {
                    linearSlideForearm.setPower(1.0);
                } else if (gamepad2.b == true) {
                    linearSlideForearm.setPower(-1.0);
                } else {
                    linearSlideForearm.setPower(0.0);
                }
                //  ARM Rotation
                if (gamepad2.left_stick_y != 0)
                    rotatingElbow.setPower(0.5);
                else if (gamepad2.left_stick_x != 0) {
                    rotatingElbow.setPower(-0.5);
                } else {
                    rotatingElbow.setPower(0);
                }

//            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

//          armAndClimb  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.c
            telemetry.addData("FL", leftBackDrive.getVelocity());
            telemetry.addData("FR", rightBackDrive.getVelocity());
            telemetry.addData("FL", leftFrontDrive.getVelocity());
            telemetry.addData("FL", rightFrontDrive.getVelocity());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("heading ", "%4.2f", heading);
            telemetry.addData("linearSlide Elevator", linearSlideElevator.getCurrentPosition());
            telemetry.addData("linearSlide Forearm", linearSlideForearm.getCurrentPosition());
            telemetry.addData("Ascend Lift Climb ", Climb.getCurrentPosition());
            telemetry.addData("rotating Robot Joint", rotatingElbow.getCurrentPosition());
            telemetry.addLine("\n");
            telemetry.addData("Sample detected", getSampleColor());

            telemetry.addData("Left distance", frontleftDistanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Right distance", frontrightDistanceSensor.getDistance(DistanceUnit.MM));

            telemetry.update();
            getSampleColor();
        }
    }

    public String getSampleColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        float maxsat = Math.max(Math.max(colors.red, colors.green), colors.blue);
        float r = colors.red/maxsat;
        float g = colors.green/maxsat;
        float b = colors.blue/maxsat;

        String sample = "Nothing";

        if (distance < 3.0) {
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
        }
        else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        return sample;
    }

// limits
 //   Linear Up - -7210, down 3330
    // Climb -
}