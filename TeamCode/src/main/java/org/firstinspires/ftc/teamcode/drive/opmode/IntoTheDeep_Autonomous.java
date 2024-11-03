package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_BASKET_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotorEx RotatingARMJoint;
    CRServo IntakeWheelSpin;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        // Set motor directions (adjust these based on your robot)
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);



        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Initialize RoadRunner drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // Drive object declared and initialized here

        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose

//        RotatingARMJoint.setDirection(DcMotorEx.Direction.FORWARD);
//        RotatingARMJoint.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        RotatingARMJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RotatingARMJoint.setTargetPosition(ROTATING_ARM_JOINT_BASKET_POSITION);
//        RotatingARMJoint.setPower(1);
//        IntakeWheelSpin.
//                IntakeWheelSpin.setPower(+1.0);


        // Define your trajectory sequence here (replace with MeepMeep output)
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
////                 Practice robot / might work if comp bot is positioned straight

//                  .forward(25)
//                .strafeLeft(-85)
//                .forward(13.2)
//                .strafeLeft(84.5)
//                .strafeLeft(-85)
//                .forward(12.5)
//                .strafeLeft(86.35)
//                .strafeLeft(-85)
//                .forward(11.5)
//                .strafeLeft(-86.3)
////               .forward(40.2)

                //Comp robot
//                .strafeLeft(12.2) //might work to drop specimen first
//                .strafeLeft(12.2) // //might work to drop specimen first
                .forward(97)
                .strafeLeft(-13.2)
                .forward(-89.65)
                .forward(89.2)
                .strafeLeft(-13.2)
                .forward(-86.7)
                .forward(87)
                .strafeLeft(-12.2)
                .forward(-86.2)
                .forward(86.2)
                .strafeLeft(12.5)







//
//
//
//
//
//
//
//
//
//






//
//
//
//                .turn(Math.toRadians(-120))
//              .strafeLeft(20)
//                .splineTo(new Vector2d(0, 36), Math.toRadians(0))
//                .forward(30)
               .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
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