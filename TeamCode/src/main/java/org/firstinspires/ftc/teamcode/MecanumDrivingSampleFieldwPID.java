package org.firstinspires.ftc.teamcode;


//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;


@TeleOp

//Example Codes for Field Centric Drive
public class MecanumDrivingSampleFieldwPID extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = true;//NOTE : FIELD CENTRIC WILL ONLY WORK IF THE CONTROL HUB IS FLAT ON THE ROBOT AT THE MOMENT!!!

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private double leftFrontDriveKp = 0;
    private double leftBackDriveKp = 0;
    private double rightFrontDriveKp = 0;
    private double rightBackDriveKp = 0;

    private double leftFrontDriveKi = 0;
    private double leftBackDriveKi = 0;
    private double rightFrontDriveKi = 0;
    private double rightBackDriveKi = 0;

    private double leftFrontDriveKd = 0;
    private double leftBackDriveKd = 0;
    private double rightFrontDriveKd = 0;
    private double rightBackDriveKd = 0;

    private double leftFrontDriveKf = 0;
    private double leftBackDriveKf = 0;
    private double rightFrontDriveKf = 0;
    private double rightBackDriveKf = 0;

    private double leftFrontDriveIntegralSum = 0;
    private double leftBackDriveIntegralSum = 0;
    private double rightFrontDriveIntegralSum = 0;
    private double rightBackDriveIntegralSum = 0;

    private DistanceSensor frontrightDistanceSensor;
    private DistanceSensor frontleftDistanceSensor;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
//      IMU calibration timer
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

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //put desired angle in radians...
        double referenceAngle = Math.toRadians(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & rotate, and right joystick to strafe.
            double axial = -gamepad1.left_stick_y;  //fwd
            double lateral = gamepad1.left_stick_x;  //TUR
            double yaw = gamepad1.right_stick_x; //STR

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

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double Adjlateral = -axial * Math.sin(heading) + lateral * Math.cos(heading);
            double Adjaxial = axial * Math.cos(heading) + lateral * Math.sin(heading);

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

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
/*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
*/
            // PID Calculation for the wheels - w/ AngleWrap method
            leftFrontPower = PIDControl(leftFrontPower, leftFrontDrive.getVelocity(), leftFrontDriveIntegralSum, leftFrontDriveKp, leftFrontDriveKi, leftFrontDriveKd, leftFrontDriveKf);
            rightFrontPower = PIDControl(rightFrontPower, rightFrontDrive.getVelocity(), rightFrontDriveIntegralSum, rightFrontDriveKp, rightFrontDriveKi, rightFrontDriveKd, rightFrontDriveKf);
            leftBackPower = PIDControl(leftBackPower, leftBackDrive.getVelocity(), leftBackDriveIntegralSum, leftBackDriveKp, leftBackDriveKi, leftBackDriveKd, leftBackDriveKf);
            rightBackPower = PIDControl(rightBackPower, rightBackDrive.getVelocity(), rightBackDriveIntegralSum, rightBackDriveKp, rightBackDriveKi, rightBackDriveKd, rightBackDriveKf);

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("non-Calibrated  Axial/Lateral", "%4.2f, %4.2f", axial, lateral);
            telemetry.addData("Calibrated  Axial/Lateral", "%4.2f, %4.2f", Adjaxial, Adjlateral);
            telemetry.addData("heading ", "%4.2f", heading);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state, double IntegralSum, double kp, double ki, double kd, double kf) {
        double error = AngleWrap(reference - state);
        IntegralSum += (error * timer.seconds());
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error * kp) + (derivative * kd) + (IntegralSum * ki) + reference * kf;
        return output;
    }

    public double AngleWrap(double radians) {
        while (radians > Math.PI)
        {
            radians -= 2*Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2*Math.PI;

        }
        return radians;
    }


}

