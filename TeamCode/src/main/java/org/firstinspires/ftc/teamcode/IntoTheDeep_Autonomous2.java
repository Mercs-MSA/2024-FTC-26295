package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_HIGH_SPECIMEN_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_LOW_BASKET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_RESET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_BASKET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_RESET_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ROTATING_ARM_JOINT_SPECIMEN_HANG_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        private DcMotorEx linearSlideElevator = null;
        private DcMotorEx Sprocket;
        private CRServo IntakeUpDown;
        private CRServo IntakeWheelSpin;
        Rev2mDistanceSensor leftDistanceSensor;
        Rev2mDistanceSensor rightDistanceSensor;
        private double originalDistance =0;
        private double targetDistance =0;
        private IMU imu = null;

        public void hangSpecimen_26295(){
            // move Robot to correct position

//      hang specimen to high rung  - Constants in RobotConstants.java file.
            linearSlideElevator.setTargetPosition(ELEVATOR_HIGH_SPECIMEN_HANG_POSITION);
            Sprocket.setTargetPosition(ROTATING_ARM_JOINT_SPECIMEN_HANG_POSITION);
            linearSlideElevator.setPower(1);
            Sprocket.setPower(1);
            // setting active holding pattern
            IntakeWheelSpin.setDirection(CRServo.Direction.REVERSE);
//            sleep(2000);
            IntakeWheelSpin.setPower(1);
//            // delay for 4 Sec
            sleep(5000);
            // reset position prior to moving the robot.
            Sprocket.setTargetPosition(ROTATING_ARM_JOINT_RESET_POSITION);
//            linearSlideElevator.setPower(1);
//            Sprocket.setPower(1);
            sleep(5000);
            IntakeWheelSpin.setPower(0);
//            linearSlideElevator.setPower(0);
//            Sprocket.setPower(0);
            linearSlideElevator.setTargetPosition(ELEVATOR_RESET_POSITION);
//            sleep(3000);


        }
        @Override
        public void runOpMode() throws InterruptedException {
            // Initialize drive motors -> Do we Need this - Vaibhav ?????
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
            //Initialize distance sensors
            leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
            rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");

            // Set motor directions (adjust these based on your robot)
            leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

            // Intake mechanism Config
            linearSlideElevator.setDirection(DcMotorEx.Direction.FORWARD);
            Sprocket.setDirection(DcMotorEx.Direction.FORWARD);

            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Intake mechanism Config
            linearSlideElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Sprocket.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            linearSlideElevator.setTargetPosition(ELEVATOR_HIGH_SPECIMEN_HANG_POSITION);
//            linearSlideElevator.setTargetPosition(linearSlideElevator.getCurrentPosition());
            linearSlideElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Sprocket.setTargetPosition(ROTATING_ARM_JOINT_SPECIMEN_HANG_POSITION);
//            Sprocket.setTargetPosition(Sprocket.getCurrentPosition());
            Sprocket.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Initialize IMU
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
            imu.initialize(parameters);

            // Initialize RoadRunner drivetrain
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // Drive object declared and initialized here

            Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose
            // Drive fwd to place specimen
            TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                    .forward(30.3)
                    .build();
            // Drive back
            TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                    .back(30.3)
                    .build();

            // Define your trajectory sequence here (replace with MeepMeep output)
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(30.3)
                    .forward(56)
                    .strafeRight(26.5)
                    .back(53)
                    .forward(53)
                    .strafeRight(22.5)
                    .back(50)
                    .forward(47)
                    .strafeRight(26)
                    .back(50)
                    .build();

            // Drive to park location
            TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(90.3)
                    .build();

            waitForStart();

            drive.followTrajectorySequence(trajSeq1);

            // Hanging Specimen holding pattern.
            hangSpecimen_26295();
            // Retract Robot
            // Realign IMU Angle to 0
            drive.followTrajectorySequence(trajSeq2);

            if (isStopRequested()) return;

            drive.followTrajectorySequence(trajSeq);
            // This is to park the Robot once all samples are pushed. - uncomment after testing
//            drive.followTrajectorySequence(trajSeq);
        }
    }
