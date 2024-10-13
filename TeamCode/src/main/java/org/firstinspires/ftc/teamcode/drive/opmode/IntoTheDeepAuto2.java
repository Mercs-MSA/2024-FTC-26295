package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class IntoTheDeepAuto2 {

    public
    static class CombinedAutonomous extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            waitForStart();

            if (isStopRequested()) return;


            // Combine movements from different tests
            Trajectory straightTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(60) // Adjust distance as needed
                    .build();

            Trajectory strafeTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(60) // Adjust distance as needed
                    .build();

            Trajectory splineTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), 0)
                    .build();

            // Follow the trajectories in sequence
            drive.followTrajectory(straightTrajectory);
            drive.followTrajectory(strafeTrajectory);
            drive.followTrajectory(splineTrajectory);
}}}
