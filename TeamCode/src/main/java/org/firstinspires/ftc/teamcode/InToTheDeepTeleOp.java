package org.firstinspires.ftc.teamcode;



import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private DcMotorEx Climb = null;
    private DcMotorEx linearSlideUpDown = null;
    private DcMotorEx linearSlideLeftRight = null;
    private DistanceSensor frontrightDistanceSensor;
    private DistanceSensor frontleftDistanceSensor;
    private Servo rollerLeftRight;
    private Servo wheelSpin;
    private Servo hook;

    
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot co
        // nfiguration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        linearSlideUpDown = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlideLeftRight = hardwareMap.get(DcMotorEx.class, "linearSlideLeftRight");
        Climb = hardwareMap.get(DcMotorEx.class, "climb");
        rollerLeftRight = hardwareMap.get(Servo. class, "rollerLeftRight");
        wheelSpin = hardwareMap.get(Servo. class, "wheelSpin");
        hook = hardwareMap.get(Servo.class, "hook");


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
       Climb.setDirection(DcMotor.Direction.FORWARD);
        linearSlideUpDown.setDirection(DcMotor.Direction.FORWARD);
       linearSlideLeftRight.setDirection(DcMotor.Direction.FORWARD);
       rollerLeftRight.setPosition(0);
       wheelSpin.setPosition(0);
       hook.setPosition(0);
//         Competition Robot Direction


        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideLeftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





/*        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
            //hook
            if (gamepad2.dpad_left){
                hook.setPosition(1);
            }
            else if (gamepad2.dpad_right){
                hook.setPosition(-1);
            }
            // Wheel SPin
           if (gamepad1.x == true){
               wheelSpin.setPosition(1);
           } else if (gamepad1.b == true) {
               wheelSpin.setPosition(-1);
           }
               else {
                   wheelSpin.setPosition(0);
               }


            // rollerLeftRight -
            if (gamepad2.right_bumper == true){
                rollerLeftRight.setPosition(1.0);

            }

             else if (gamepad2.left_bumper == true) {
                rollerLeftRight.setPosition(-1.0);
            }
            else {
                rollerLeftRight.setPosition(0.0);

// Linear Slide Intake Up and Down
            if (gamepad2.x == true){
                linearSlideUpDown.setPower(0.5);

            }
            else if (gamepad2.y == true) {
                linearSlideUpDown.setPower(-1.0);
            }
            else {
                linearSlideUpDown.setPower(0.0);
            }
// Linear Slide Left Right\
            if (gamepad2.a == true){
                    linearSlideLeftRight.setPower(1.0);
                }
            else if (gamepad2.b == true) {
                    linearSlideLeftRight.setPower(-1.0);
                }
            else {
                    linearSlideLeftRight.setPower(0.0);

//             Below this is code to get the arm and climb working
            if ( gamepad2.dpad_up == true){
            Climb.setPower(1.0);

            }
            else if (gamepad2.dpad_down == true) {
                Climb.setPower(-1.0);
            }
            else {
                Climb.setPower(0.0); // remember to turn off if nothing pressed!
//
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

            //armAndClimb  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("FL", leftBackDrive.getVelocity());
            telemetry.addData("FR", rightBackDrive.getVelocity());
            telemetry.addData("FL", leftFrontDrive.getVelocity());
            telemetry.addData("FL", rightFrontDrive.getVelocity());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("non-Calibrated  Axial/Lateral", "%4.2f, %4.2f", axial, lateral);
            telemetry.addData("Calibrated  Axial/Lateral", "%4.2f, %4.2f", Adjaxial, Adjlateral);
            telemetry.addData("heading ", "%4.2f", heading);
            telemetry.update();
        }
    }


}}}}