package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous
public class AutonomousWithDashboardPreview extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize drive motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");


        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize RoadRunner drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize Dashboard and Camera Stream
        FtcDashboard dashboard = FtcDashboard.getInstance();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // Replace "Webcam 1" if needed
        //dashboard.startCameraStream((CameraStreamSource) webcamName, 30);
        dashboard.startCameraStream((CameraStreamSource) webcamName, 30);
        detectCameras(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0)); //  start pose

        // Define your trajectory sequence here (replace with MeepMeep output)
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(24)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

    }

    // Function to detect and list connected cameras
    private void detectCameras(HardwareMap hardwareMap) {
        List<WebcamName> webcamNames = hardwareMap.getAll(WebcamName.class);

        if (webcamNames.isEmpty()) {
            telemetry.addData("Camera Status", "No cameras detected");
        } else {
            telemetry.addData("Camera Status", "Connected cameras:");
            for (WebcamName webcamName : webcamNames) {
                telemetry.addData("  -", webcamName.toString());
            }
        }

        telemetry.update();
    }
}

