package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_LOW_BASKET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_RESET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_BASKET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_RESET_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class IntoTheDeep_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private IMU imu = null;

    private DcMotorEx linearSlideElevator = null;
    private DcMotorEx Sprocket;
    private CRServo IntakeUpDown;
    private CRServo IntakeWheelSpin;

    public void hangSpecimen_26295(){
        // move Robot to correct position

//      hang specimen to high rung  - Constants in RobotConstants.java file.
        linearSlideElevator.setPower(1);
        linearSlideElevator.setTargetPosition(ELEVATOR_LOW_BASKET_POSITION);
        Sprocket.setTargetPosition(ROTATING_ARM_JOINT_BASKET_POSITION);
        linearSlideElevator.setPower(1);
        Sprocket.setPower(1);
//        Sprocket.setPower(1);
//            IntakeWheelSpin.setDirection(CRServo.Direction.REVERSE);
        // delay for 2 Sec - optimize after testing.
//            linearSlideElevator.setPower(1);
//            Sprocket.setPower(1);
//            sleep(2000);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        // Intake Mechanism Init
        linearSlideElevator = hardwareMap.get(DcMotorEx.class, "linearSlideElevator");
        Sprocket = hardwareMap.get(DcMotorEx.class, "Sprocket");
        //Servos
        IntakeUpDown = hardwareMap.get(CRServo.class,"IntakeUpDown");
        IntakeWheelSpin = hardwareMap.get(CRServo.class, "WheelSpin");

        // Set motor directions (adjust these based on your robot)
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake mechanism Init & Config
        linearSlideElevator.setDirection(DcMotorEx.Direction.FORWARD);
        linearSlideElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Sprocket.setDirection(DcMotorEx.Direction.FORWARD);
        Sprocket.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideElevator.setTargetPosition(ELEVATOR_LOW_BASKET_POSITION);
        linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Sprocket.setTargetPosition(ROTATING_ARM_JOINT_BASKET_POSITION);
        Sprocket.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Initialize RoadRunner drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // Drive object declared and initialized here

        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose

        // Move fwd to drop the sample in basket.
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                //Comp robot- most probably will NOT work
                .forward(15) //might work to drop specimen first
                .build();

        //Retract to follow remaining auto path
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                //Comp robot- most probably will NOT work
                .back(11.25) //might work to drop specimen first
                .build();

        // Define your trajectory sequence here (replace with MeepMeep output)
        //strafe 3 samples in netzone.
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //Comp robot- most probably will NOT work
//               .strafeLeft(-18.2) //might work to drop specimen first
                .strafeRight(85.2)
                .turn(0.95)
                .forward(22)
                .strafeRight(20.5)
                .forward(22.5)
                .strafeLeft(85.452)
                .back(1.5)
                .strafeRight(85.2)
                .forward(22.5)
                .strafeLeft(83.2)

//                .turn(-6.2)
//                .strafeLeft(10.3)
//                .forward(89.2)
//                .strafeLeft(-11.6)
//                .forward(-88.7)
//                .forward(87)
//                .strafeLeft(-11.5)
//                .forward(-86.2)
//                .forward(86.2)
//                .strafeLeft(12.5)
//                .forward(30.5)
//                .strafeLeft(10.2)
                .build();
        // Define parking trajectory in app note
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startPose)
                //Comp robot- most probably will NOT work
                .strafeRight(95)
                .back(25.2)
                .build();

        waitForStart();
        hangSpecimen_26295();
        // Hanging Specimen holding pattern.
        drive.followTrajectorySequence(trajSeq1);

        sleep(500);
        IntakeWheelSpin.setPower(1);
        // delay for 1 Sec to split the sample out
        sleep(500);
        // reset position prior to moving the robot.
//            linearSlideElevator.setPower(0);
//            Sprocket.setPower(0);
        linearSlideElevator.setTargetPosition(ELEVATOR_RESET_POSITION);
        Sprocket.setTargetPosition(ROTATING_ARM_JOINT_RESET_POSITION);
        IntakeWheelSpin.setPower(0);

        // go fwd to actually hang specimen
        drive.followTrajectorySequence(trajSeq2);


        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
        // uncomment after testing - this is to park the Bot to parking positon in basket side Auto
        drive.followTrajectorySequence(trajSeq4);

    }
}
//Distance for Auton 1 start position: 704
// Lift arm for Auton 1 reset position: 0
// ;linear slide elevator position : -6
//linear slide arm position :126
//lower basket position rotating arm :278
//lower basket position linear slide : 116
//left distance senson : 540
//wheel rotarion : no value
// linear slide elevation : -6