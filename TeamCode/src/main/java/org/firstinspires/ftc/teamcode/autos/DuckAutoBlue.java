package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.level1;
import static org.firstinspires.ftc.teamcode.Constants.level2;
import static org.firstinspires.ftc.teamcode.Constants.level3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RoadRunner Carousel Auto Blue", group="Autonomous")
public class DuckAutoBlue extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.BLUE);
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();
        Webcam webcam = new Webcam();

        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);
        webcam.init(hardwareMap);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                .strafeRight(1.5)
                .back(12)
                .turn(Math.toRadians(40))
                .back(19)
                .build();
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .forward(28)
                .turn(Math.toRadians(-130))
                .back(15)
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .forward(5)
                .strafeLeft(5.35)
                .forward(90)
                .build();

        delay(500);
        // Get the placement of the shipping element 100 times and pick the most frequent position
        int level;
        int[] counts = {0,0,0};
        for(int i=0;i<150;i++) {
            delay(10);
            if(webcam.getShippingHubLevel() == 0) {
                i = 0;
                continue;
            }
            counts[webcam.getShippingHubLevel() - 1] += 1;
        }

        if(counts[0] > counts[1] && counts[0] > counts[2]) {
            level = 1;
        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
            level = 2;
        } else {
            level = 3;
        }

        waitForStart();

        drive.followTrajectorySequence(trajectory1);
        if (level == 1) {
            lift.goTo(level1, 0.8);
        } else if (level == 2) {
            lift.goTo(level2, 0.8);
        } else if (level == 3) {
            lift.goTo(level3, 0.8);
        } else {
            throw new IllegalStateException("Invalid shipping hub level: " + level);
        }
        delay(750);
        hopper.hopper.setPosition(0.33);
        delay(700);
        hopper.hopper.setPosition(0);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(trajectory2);
        carousel.turnCarousel();
        delay(2500);
        drive.followTrajectorySequence(trajectory3);


        //                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55, 55), Math.toRadians(135))
//                        .waitSeconds(1)
//                        .forward(8)
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(-24, 37), Math.toRadians(-45))
//                        .setReversed(false)
//                        .waitSeconds(1)
//                        .splineTo(new Vector2d(44, 64), Math.toRadians(0))
    }
    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
