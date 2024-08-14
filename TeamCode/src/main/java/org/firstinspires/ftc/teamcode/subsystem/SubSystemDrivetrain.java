package org.firstinspires.ftc.teamcode.subsystem;
//package org.firstinspires.ftc.teamcode;

// Mecanum Drivetrain

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SubSystemVariables;

import java.lang.Math;

public class SubSystemDrivetrain {
    //Which robot are we on? Hubs are mounted different on A & B
    // Instantiate the drivetrain motor variables
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx backRightDrive;
    private DistanceSensor frontDistanceSensor;
    private IMU imu;
    private double ZeroAngleOffsetRads  = 0.0;
    private double ZeroAngleOffsetDegs  = 0.0;
    private double speedDeadband = 0.05;
    public static double FLP = 0.0;
    public static double FRP = 0.0;
    public static double BLP = 0.0;
    public static double BRP = 0.0;
    private final double sqrt2   = Math.sqrt(2);
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable4

    public SubSystemDrivetrain(HardwareMap hardwareMap, int currentBot) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");      //Sets the names of the hardware on the hardware map
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");      // "DeviceName" must match the Config EXACTLY
        backLeftDrive   = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        // Motors on one side need to effectively run 'backwards' to move 'forward'
        // Reverse the motors that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        //Front distance sensor
        if (currentBot == 0)
            frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");

        if (currentBot == 0) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            );
        }
        else {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            );
        }
    }

    private void resetEncoders(){
        //Stop the motors and reset the encoders to zero
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //Make sure we re-enable the use of encoders
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void resetGyro(){
        imu.resetYaw();
    }

    private int[] getMotorEncoders(){
        int[] result = new int[4];

        result[0] = frontLeftDrive.getCurrentPosition();
        result[1] = frontRightDrive.getCurrentPosition();
        result[2] = backLeftDrive.getCurrentPosition();
        result[3] = backRightDrive.getCurrentPosition();

        return result;
    }

    public double getCurrentHeading(boolean resultInRads){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if (resultInRads)
            return loopInRange(orientation.getYaw(AngleUnit.RADIANS) - ZeroAngleOffsetRads, Math.PI);
        else
            return loopInRange(orientation.getYaw(AngleUnit.DEGREES) - ZeroAngleOffsetDegs, 180);
    }

    public double loopInRange(double value, double range)
    {
        while(value > range) { value -= (range + range);}
        while(value < -range) { value += (range + range);}
        return value;
    }

    public void disableDrivetrainMotors()
    {
        setMotors(0, 0, 0, 0);
    }

    public void setMotors(double FL, double FR, double BL, double BR){
        double max;
        //If any value is greater than 1.0 normalize all values accordingly
        max = Math.max(Math.max(FL,FR),Math.max(BL,BR));
        if (max > 1.0){
            FL = FL / max;
            FR = FR / max;
            BL = BL / max;
            BR = BR / max;
        }

        FLP = FL;
        FRP = FR;
        BLP = BL;
        BRP = BR;

        frontLeftDrive.setPower(FL);
        frontRightDrive.setPower(FR);
        backLeftDrive.setPower(BL);
        backRightDrive.setPower(BR);

    }

    /**
     * Calculate normalization factor angle (i.e. where in semi-quadrant for unit square to unit circle transposition)
     * @param angle angle to calculate normalization factor for in degrees
     */
    private double unitNormalizationAngle(double angle){
        double normalizationAngle;
        double angleDegrees;

        angleDegrees = Math.toDegrees(angle);
        if (angleDegrees >= 0)
            normalizationAngle = angleDegrees % 90;
        else
            normalizationAngle = (-angleDegrees) % 90;

        if (normalizationAngle >= 45) {
            normalizationAngle = 90 - normalizationAngle;
        }

        //
        return normalizationAngle;
    }

    public void doMecanumDrive(double translateSpeed, double heading, double rotation, boolean fieldCentric)
    {
        double finalSpeed;
        double normalizationFactor;
        double normalizationAngle;
        boolean normalize = false;
        double currentHeading = 0.0;
        double botCentricDirection;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        if (translateSpeed < speedDeadband){//Create deadband and also ensure no divide by zero in atan2 and stop robot twitching

            heading = 0;
            translateSpeed = 0;
        }

        if (normalize) {
            //This version will normalize the joystick position to fully utilize the entire power range for 0, 90, 180 & 270 by mapping from unit square to unit circle
            //Adjust to use full range of speed to 'boost' the power for 0, 90, 180, 270 from .707 to 1.0
            normalizationAngle = unitNormalizationAngle(heading);
            normalizationFactor = (Math.sqrt(1 + Math.tan(normalizationAngle)) / sqrt2);
            finalSpeed = translateSpeed / (normalizationFactor * sqrt2);
        }
        else {
            //This version is a simplified version that does not maximize the entire power range of the motors
            finalSpeed = translateSpeed;
        }
        if (fieldCentric) {
            currentHeading = getCurrentHeading(true);
        }
        botCentricDirection = currentHeading - heading;

        //Note, the diagonally opposite speeds are the same for pure translation, so only need to calculate 2 values
        frontLeftSpeed  = finalSpeed*Math.sin((botCentricDirection) + (Math.PI/4));
        frontRightSpeed = finalSpeed*Math.cos((botCentricDirection) + (Math.PI/4));

        backRightSpeed = frontLeftSpeed;
        backLeftSpeed = frontRightSpeed;

        //Add in any desired rotation
        frontLeftSpeed = frontLeftSpeed   + rotation;
        backLeftSpeed  = backLeftSpeed    + rotation;
        frontRightSpeed = frontRightSpeed - rotation;
        backRightSpeed  = backRightSpeed  - rotation;

        //And actually set the motors accordingly
        //Note, this function will also clamp and scale the power to 1.0
        setMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void setSpeedDeadband(double deadband)
    {
        speedDeadband = deadband;
    }

    public void turnHeading(double maxTurnSpeed, double heading) {
        double turnSpeed;
        // Determine required steering to keep on heading
        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

        // Clip the speed to the maximum permitted value.
        turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

        // Pivot in place by applying the turning correction
        moveRobot(0, turnSpeed);
    }

    public void driveHeading(double speed, double maxTurnSpeed, double heading) {
        double turnSpeed;
        // Determine required steering to keep on heading
        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
        // Clip the speed to the maximum permitted value.
        turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
        moveRobot(speed, turnSpeed);

    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading;
        double headingError;

        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getCurrentHeading(false);//In degrees

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        setMotors(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }

    public double getFrontDistanceSensor(){
        return frontDistanceSensor.getDistance(DistanceUnit.MM);
    }

}