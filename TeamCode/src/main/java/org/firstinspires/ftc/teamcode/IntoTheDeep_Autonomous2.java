package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


    @Autonomous
    public class IntoTheDeep_Autonomous2 extends LinearOpMode {
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotorEx leftFrontDrive = null;
        private DcMotorEx leftBackDrive = null;
        private DcMotorEx rightFrontDrive = null;
        private DcMotorEx rightBackDrive = null;
        private IMU imu = null;

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

            // Define your trajectory sequence here (replace with MeepMeep output)
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .forward(25)
//                .forward(85)
//                .strafeLeft(13.2)
//                .forward(-86.5)
//                .forward(85)
//                .strafeLeft(12.5)
//                .forward(-86.35)
//                .forward(84.7)
//                .strafeLeft(11.8)
//                    .forward(-86.3)
////                    .forward(40.2)

//                .turn(Math.toRadians(-120))
//              .strafeLeft(20)
//                .splineTo(new Vector2d(0, 36), Math.toRadians(0))
//                .forward(30)
                    .strafeLeft(-32)
                    .strafeLeft(31.2)
                    .forward(51.3)
                    .strafeLeft(-55)
                    .forward(28)
                    .strafeLeft(53)
                    .strafeLeft(-55)
                    .forward(27.3)
                    .strafeLeft(53)
                    .strafeLeft(-54)
                    .forward(28)
                    .strafeLeft(55)
                    .strafeLeft(-55)
                    .forward(-51.3)
                    .strafeLeft(-55)
                    .forward(30)








                    .build();


            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectorySequence(trajSeq);
        }
    }
