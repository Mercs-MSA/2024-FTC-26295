package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.HzArm;
import org.firstinspires.ftc.teamcode.SubSystems.HzAutoControl;
import org.firstinspires.ftc.teamcode.SubSystems.HzDrive;
import org.firstinspires.ftc.teamcode.SubSystems.HzGameField;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzIntake;
import org.firstinspires.ftc.teamcode.SubSystems.HzLaunchController;
import org.firstinspires.ftc.teamcode.SubSystems.HzLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.HzMagazine;
import org.firstinspires.ftc.teamcode.SubSystems.HzVuforia;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


/**
 * Ultimate Goal Autonomous mode <BR>
 *
 * This code describes how Autonomous mode is done by Hazmat Robot for Ultimate Goal.<BR>
 * The following options are coded here, and selectable through Gamepad inputs to set up <BR>
 *     <emsp>Playing Alliance : Red or Blue</emsp>
 *     <emsp>Start Line : Inner or Outer</emsp>
 *     <emsp>Game options :</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch, move Wobble Goal2 and park</emsp>
 *
 * The code for Red and Blue are written as reflection of each other.<BR>
 * Camera on either side is used using Vuforia to determine target for Wobble Goal<BR>
 */
@Autonomous(name = "Hazmat Autonomous", group = "00-Autonomous" , preselectTeleOp = "Hazmat TeleOp RR")
public class HzAutonomousBasic extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    public HzGamepad hzGamepad;
    public HzAutoControl hzAutoControl;
    public HzDrive hzDrive;
    public HzMagazine hzMagazine;
    public HzIntake hzIntake;
    public HzLaunchController hzLaunchController;
    public HzLauncher hzLauncher;
    public HzArm hzArm;

    public HzVuforia hzVuforia;
    public Pose2d startPose = HzGameField.BLUE_INNER_START_LINE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public HzGameField.TARGET_ZONE targetZone = HzGameField.TARGET_ZONE.A;
    public HzVuforia.ACTIVE_WEBCAM activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;

    double af = HzGameField.ALLIANCE_FACTOR;

    double turnAnglePowershot12 = Math.toRadians(-5);
    double turnAnglePowershot23 = Math.toRadians(-5);
    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize HW
        hzDrive = new HzDrive(hardwareMap);
        hzMagazine = new HzMagazine(hardwareMap);
        hzIntake = new HzIntake(hardwareMap);

        hzLauncher = new HzLauncher(hardwareMap);
        hzArm = new HzArm(hardwareMap);
        hzLaunchController = new HzLaunchController(hardwareMap, hzLauncher, hzIntake, hzMagazine, hzDrive);
        hzGamepad = new HzGamepad(gamepad1,hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);
        hzAutoControl = new HzAutoControl(hzDrive,hzMagazine,hzIntake,hzLaunchController,hzLauncher,hzArm);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
        hzVuforia = new HzVuforia(hardwareMap, activeWebcam);
        af = HzGameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        hzVuforia.activateVuforiaTensorFlow();

        hzDrive.getLocalizer().setPoseEstimate(startPose);

        hzIntake.setIntakeReleaseHold();
        hzAutoControl.setMagazineToLaunch();

        while (hzMagazine.magazineLaunchTouchSensor.isPressed() == false) {
            hzAutoControl.setMagazineToLaunch();
            if (isStopRequested()) return;
        }

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = hzVuforia.runVuforiaTensorFlow();

            if (!parked){
                hzAutoControl.setMagazineToLaunch();
                hzAutoControl.runAutoControl();
            }

            if (HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
            hzLaunchController.launchMode = HzLaunchController.LAUNCH_MODE.MANUAL;

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                hzVuforia.deactivateVuforiaTensorFlow();

                if (HzGameField.startPosition == HzGameField.START_POSITION.INNER) {
                    runAutoInner();
                } else { //HzGameField.startPosition == HzGameField.START_POSITION.OUTER
                    runAutoOuter();
                }

                hzIntake.setIntakeReleaseOpen();
                hzAutoControl.setMagazineToCollect();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                HzGameField.currentPose = hzDrive.getPoseEstimate();
                HzGameField.poseSetInAutonomous = true;

                if (HzDEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        HzGameField.currentPose = hzDrive.getPoseEstimate();
        HzGameField.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoInner(){

        // Set magazine to Launch in case it slipped
        hzAutoControl.setMagazineToLaunch();

        // For Launch and Park mode, wait time, so that alliance robot is not crossed
        if (!hzAutoControl.dropFirstWobbleGoal){
            hzWait(20000);
        }

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
            hzAutoControl.setLaunchTargetHighGoal();

            //Intermediary position to move away from alliance robot.
            //For option of second wobble goal drop, this is avoided to save time
            if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                        .build();
                hzDrive.followTrajectory(traj);
            }
            hzAutoControl.setMagazineToLaunch();
            //Move to position to launch rings
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*14,Math.toRadians(af*15))) //18
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutoControl.setLaunchTargetPowerShot1();

            //Intermediary position to move away from alliance robot.
            //For option of second wobble goal drop, this is avoided to save time
            if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-34, af * 14, Math.toRadians(af * 45))) //-40,6
                        .build();
                hzDrive.followTrajectory(traj);
            }
            //Move to position to launch rings
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*14,Math.toRadians(af*5)))
                    .build();
            hzDrive.followTrajectory(traj);

            //Set turn angles prior to launching for each of the power shorts
            turnAnglePowershot12 = Math.toRadians(af*-5);
            turnAnglePowershot23 = Math.toRadians(af*-5);
            launch3RingsToPowerShots();
        }

        if (!hzAutoControl.dropFirstWobbleGoal){
            //Park after launch rings
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 20, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
            return;
        } else {
            // Move to drop wobble goal on target
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(27, af * 48, Math.toRadians(af * -45)))//43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(22, af * 26, Math.toRadians(af * -135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(54, af * 39, Math.toRadians(af * -90))) //y:51
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(34, af * 40, Math.toRadians(af * -45)))//43
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(17, af * 15, Math.toRadians(af * -135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(54, af * 39, Math.toRadians(af * -90))) //y:51
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                }
            }

            dropWobbleGoalInTarget();

            if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)) { // Park
                //Park
                if (hzAutoControl.pickAndDropSecondWobbleGoal) {
                    runInnerPickAndDropSecondWobbleGoalAndPark();
                } else {
                    //Move towards base line away from other robot and Park
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(50, af * 16, Math.toRadians(af * -45)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af * 20, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                }

            /* Alternate
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(0)))
                    .build();
            hzDrive.followTrajectory(traj);
            */
                return;
            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                hzIntake.setIntakeReleaseOpen();
                hzAutoControl.setIntakeStart();
                hzWait(500);

                //Move to Position to pick rings
                if (targetZone == HzGameField.TARGET_ZONE.B) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 36, Math.toRadians(af * 180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-22, af * 36, Math.toRadians(af * 180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    hzWait(300);
                }

                // Spline to (24,24,0)
                if (targetZone == HzGameField.TARGET_ZONE.C) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 36, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    /*if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-22, af * 37, Math.toRadians(af * -180)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }*/

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-27, af * 36, Math.toRadians(af * -180))) //x33
                            .build();
                    hzDrive.followTrajectory(traj);

                    hzWait(300);
                }

                if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutoControl.setIntakeStop();
                    hzAutoControl.setMagazineToLaunch();
                    hzAutoControl.setLaunchTargetHighGoal();

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-13, af * 36, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    launch3RingsToHighGoal();

                    if (hzAutoControl.pickAndDropSecondWobbleGoal == true) {
                        runInnerPickAndDropSecondWobbleGoalAndPark();
                    } else {
                        //Park
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(10, af * 0, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    hzAutoControl.setIntakeStop();
                }
            }
        }
    }

    /**
     * Path for picking and dropping second wobble goal for Inner Start position
     */
    public void runInnerPickAndDropSecondWobbleGoalAndPark() {
        hzAutoControl.setMoveArmPickWobble();
        hzAutoControl.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-33, af*50, Math.toRadians(af*0)))
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(200);
        hzAutoControl.runCloseGrip();
        hzWait(400);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(42, af*50, Math.toRadians(af*-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameField.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    /**
     * Path and actions for autonomous mode starting from Outer start position
     */
    public void runAutoOuter(){
        // Set magazine to Launch in case it slipped
        hzAutoControl.setMagazineToLaunch();

        if (!hzAutoControl.dropFirstWobbleGoal){
            hzWait(20000);
        }

        // Move to launch position and launch rings to High Goal or Powershots
        if (hzAutoControl.autoLaunchAim == HzAutoControl.AutoLaunchAim.HIGHGOAL) {
            hzAutoControl.setLaunchTargetHighGoal();
            if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * -45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            hzAutoControl.setMagazineToLaunch();
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10,af*50,Math.toRadians(af*-8)))//-10
                    .build();
            hzDrive.followTrajectory(traj);

            launch3RingsToHighGoal();
        } else {
            hzAutoControl.setLaunchTargetPowerShot1();
            if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, af * 50, Math.toRadians(af * 45)))
                        .build();
                hzDrive.followTrajectory(traj);
            }
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-10,af*50,Math.toRadians(af*-15)),Math.toRadians(0))
                    .build();
            hzDrive.followTrajectory(traj);

            //Set turn angles prior to launching
            turnAnglePowershot12 = Math.toRadians(-5);
            turnAnglePowershot23 = Math.toRadians(-5);
            launch3RingsToPowerShots();
        }

        if (!hzAutoControl.dropFirstWobbleGoal) {
            //Park
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 40, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
            return;
        } else {

            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-10, 48, Math.toRadians(-135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(15, 48, Math.toRadians(135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(39, 52, Math.toRadians(-150)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;

                }
            } else { //HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE
                switch (targetZone) {
                    case A:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-10, -48, Math.toRadians(135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case B:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(15, af * 48, Math.toRadians(135)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;
                    case C:
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(40, af * 44, Math.toRadians(180)))
                                .build();
                        hzDrive.followTrajectory(traj);
                        break;

                }
            }

            dropWobbleGoalInTarget();

            if ((hzAutoControl.pickRingFromTargetMarker == false) || (targetZone == HzGameField.TARGET_ZONE.A)) { // Park
                if (hzAutoControl.pickAndDropSecondWobbleGoal) {
                    runOuterPickAndDropSecondWobbleGoalAndPark();
                } else {
                    //Park
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(13, af * 20, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);

            /* Alternate
            traj =  hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-50,af*16,Math.toRadians(af*-90)))
                    .build();
            hzDrive.followTrajectory(traj);
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(10, af*0, Math.toRadians(af*0)))
                    .build();
            hzDrive.followTrajectory(traj);
            */
                }
                return;

            } else { //Target Zone is B or C and pickRingFromTargetMarker == True
                //Pick rings from Target Marker
                hzIntake.setIntakeReleaseOpen();
                hzAutoControl.setIntakeStart();
                hzWait(500);

                //Move to Position to pick rings
                if (targetZone == HzGameField.TARGET_ZONE.B) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 36, Math.toRadians(af * 180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-22, af * 36, Math.toRadians(af * 180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    hzWait(300);
                }

                // Spline to (24,24,0)
                if (targetZone == HzGameField.TARGET_ZONE.C) {
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-10, af * 36, Math.toRadians(af * -180)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    /*if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-22, af * 37, Math.toRadians(af * -180)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }*/

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-27, af * 36, Math.toRadians(af * -180))) //x33
                            .build();
                    hzDrive.followTrajectory(traj);

                    hzWait(300);
                }

                if (hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal) {
                    hzAutoControl.setIntakeStop();
                    hzAutoControl.setMagazineToLaunch();
                    hzAutoControl.setLaunchTargetHighGoal();

                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-13, af * 40, Math.toRadians(af * 3)))
                            .build();
                    hzDrive.followTrajectory(traj);

                    launch3RingsToHighGoal();

                    if (hzAutoControl.pickAndDropSecondWobbleGoal == true) {
                        runOuterPickAndDropSecondWobbleGoalAndPark();
                    } else {

                        //Park
                        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13, af * 52, Math.toRadians(af * 0)))
                                .build();
                        hzDrive.followTrajectory(traj);
                    }
                } else { //Move to baseline and park in safe zone
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-50, af * 16, Math.toRadians(af * -90)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(10, af * 0, Math.toRadians(af * 0)))
                            .build();
                    hzDrive.followTrajectory(traj);
                    hzAutoControl.setIntakeStop();
                }
            }
        }
    }

    /**
     * Path for picking and dropping second wobble goal for Outer Start position
     */
    public void runOuterPickAndDropSecondWobbleGoalAndPark() {
        hzAutoControl.setMoveArmPickWobble();
        hzAutoControl.runOpenGrip();
        traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-31, af*23, Math.toRadians(af*0)))
                .build();
        hzDrive.followTrajectory(traj);
        hzWait(200);
        hzAutoControl.runCloseGrip();
        hzWait(400);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzWait(500);
        switch (targetZone) {
            case A:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*20, Math.toRadians(af*-60)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case B:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(13, af*33, Math.toRadians(af*180)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
            case C:
                traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(37, af*50, Math.toRadians(af*-135)))
                        .build();
                hzDrive.followTrajectory(traj);
                break;
        }
        dropWobbleGoalInTarget();
        //Park
        if (targetZone == HzGameField.TARGET_ZONE.C) {
            traj = hzDrive.trajectoryBuilder(hzDrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(13, af * 33, Math.toRadians(af * 0)))
                    .build();
            hzDrive.followTrajectory(traj);
        }
    }

    /**
     * Sequence of launching 3 rings to High Goal
     */
    public void launch3RingsToHighGoal(){
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(200);
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450); //400
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450);
        hzAutoControl.setRunLauncherTrue();
        hzAutoControl.setMagazineToLaunch();
        hzWait(450);
        hzAutoControl.setRunLauncherTrue();

        // Run 4th just in case
        if (!hzAutoControl.pickAndDropSecondWobbleGoal) {
            hzAutoControl.setMagazineToLaunch();
            hzWait(450);
            hzAutoControl.setRunLauncherTrue();
        }
        hzWait(600);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    public void launch3RingsToHighGoal1(){
        int counter=0;
        hzAutoControl.setLaunchTargetHighGoal();
        hzAutoControl.setMagazineToLaunch();
        hzWait(200);
        hzAutoControl.setLaunchTargetHighGoal();
        for (counter = 0; counter <7; counter++) {
            hzAutoControl.setMagazineToLaunch();
            hzWait(250); //400
            hzAutoControl.setRunLauncherTrue();
        };

        hzWait(500);
        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();
    }

    /**
     * Sequence of launching 3 rings to PowerShots
     */
    public void launch3RingsToPowerShots(){
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(600);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot12);
        hzAutoControl.setLaunchTargetPowerShot1();
        hzAutoControl.setMagazineToLaunch();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();

        hzDrive.turn(turnAnglePowershot23);
        hzAutoControl.setMagazineToLaunch();
        hzAutoControl.setLaunchTargetPowerShot1();
        hzWait(400);
        hzAutoControl.setRunLauncherTrue();
        hzWait(400);

        hzAutoControl.setLaunchTargetOff();
        hzAutoControl.setMagazineToCollect();

    }

    /**
     * Sequence of dropping Wobble Goal once it reaches target position
     */
    public void dropWobbleGoalInTarget(){
        hzAutoControl.setMoveArmDropWobbleAutonoumous();
        //hzWait(1000);
        hzWait(500);
        hzAutoControl.runOpenGrip();
        hzWait(300);
        hzAutoControl.setMoveArmHoldUpWobbleRing();
        hzAutoControl.setMoveArmParked();
    }


    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void hzWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            hzAutoControl.runAutoControl();
        }
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:47 :: 2/13");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();


        while (!isStopRequested()) {
            if (hzGamepad.getButtonBPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                HzGameField.playingAlliance = HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                HzGameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        hzWait(200);

        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :", "(Inner: (A) ,    Outer: (Y))");
        while (!isStopRequested()) {
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.INNER;
                    startPose = HzGameField.RED_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "RED_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
                    startPose = HzGameField.RED_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "RED_OUTER_START_LINE");
                    break;
                }
            }
            if (HzGameField.playingAlliance == HzGameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (hzGamepad.getButtonAPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.INNER;
                    startPose = HzGameField.BLUE_INNER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.RIGHT;
                    telemetry.addData("Start Pose : ", "BLUE_INNER_START_LINE");
                    break;
                }
                if (hzGamepad.getButtonYPress()) {
                    HzGameField.startPosition = HzGameField.START_POSITION.OUTER;
                    startPose = HzGameField.BLUE_OUTER_START_LINE;
                    activeWebcam = HzVuforia.ACTIVE_WEBCAM.LEFT;
                    telemetry.addData("Start Pose : ", "BLUE_OUTER_START_LINE");
                    break;
                }
            }
            telemetry.update();
        }
        telemetry.update();


        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ",activeWebcam);
            telemetry.addLine();
            telemetry.addData("Please select launch target : ", "(X) for Powershot, (B) for High Goal, ");
            if (hzGamepad.getButtonBPress()) {
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.HIGHGOAL;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.autoLaunchAim = HzAutoControl.AutoLaunchAim.POWERSHOT;
                telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
                break;
            }
            telemetry.update();
            hzWait(200);
        }

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance :", HzGameField.playingAlliance);
            telemetry.addData("startPose : ", startPose);
            telemetry.addData("activeWebcam : ", activeWebcam);
            telemetry.addData("hzAutoControl.autoLaunchAim : ", hzAutoControl.autoLaunchAim);
            telemetry.addLine();
            telemetry.addData("Please select option for Picking rings from target markers and launching : ", "");
            telemetry.addData("    (Y):","Launch and park");
            telemetry.addData("    (A):","Launch, drop WG and park");
            telemetry.addData("    (X):","Launch, drop WG, Pick rings, launch and park");
            telemetry.addData("    (B):","Launch, drop WG, Pick rings, launch, move WG2 and park");

            if (hzGamepad.getButtonYPress()) {
                hzAutoControl.dropFirstWobbleGoal = false;
                hzAutoControl.pickRingFromTargetMarker = false;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                hzAutoControl.pickAndDropSecondWobbleGoal = false;
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                break;
            }

            if (hzGamepad.getButtonAPress()) {
                hzAutoControl.dropFirstWobbleGoal = true;
                hzAutoControl.pickRingFromTargetMarker = false;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = false;
                hzAutoControl.pickAndDropSecondWobbleGoal = false;
                telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                break;
            }
            if (hzGamepad.getButtonXPress()) {
                hzAutoControl.dropFirstWobbleGoal = true;
                hzAutoControl.pickRingFromTargetMarker = true;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;
                hzAutoControl.pickAndDropSecondWobbleGoal = false;
                telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                break;
            }
            if (hzGamepad.getButtonBPress()) {
                hzAutoControl.dropFirstWobbleGoal = true;
                hzAutoControl.pickRingFromTargetMarker = true;
                hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal = true;
                hzAutoControl.pickAndDropSecondWobbleGoal = true;
                telemetry.addData("hzAutoControl.dropFirstWobbleGoal : ", hzAutoControl.dropFirstWobbleGoal);
                telemetry.addData("hzAutoControl.pickRingFromTargetMarker : ", hzAutoControl.pickRingFromTargetMarker);
                telemetry.addData("hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal  : ", hzAutoControl.launchRingsPickedFromTargetMarkerToHighGoal);
                telemetry.addData("hzAutoControl.pickAndDropSecondWobbleGoal: ", hzAutoControl.pickAndDropSecondWobbleGoal);
                break;
            }
            telemetry.update();
        }

        telemetry.update();
        sleep(200);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("autonomousStarted : ", autonomousStarted);

        telemetry.addData("GameField.playingAlliance : ", HzGameField.playingAlliance);
        telemetry.addData("startPose : ", startPose);
        telemetry.addData("HzGameField.ALLIANCE_FACTOR",HzGameField.ALLIANCE_FACTOR);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", hzDrive.driveMode);
        telemetry.addData("PoseEstimate :", hzDrive.poseEstimate);
        telemetry.addData("HzGameField.currentPose",HzGameField.currentPose);

        //telemetry.addData("Visible Target : ", hzVuforia.visibleTargetName);
        telemetry.addData("hzVuforia.targetZoneDetected", hzVuforia.targetZoneDetected);
        telemetry.addData("targetZone :", targetZone);
        // Print pose to telemetry
        //telemetry.addData("PoseVuforia :",hzVuforia1.poseVuforia);

        //******* Magazine Debug ********
        switch (hzMagazine.getMagazinePosition()) {
            case AT_LAUNCH: {
                telemetry.addData("hzMagazine.getMagazinePosition(): ", "MAGAZINE_AT_LAUNCH");
                break;
            }
            case AT_COLLECT: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_COLLECT");
                break;
            }
            case AT_ERROR: {
                telemetry.addData("hzMagazine.getMagazinePosition():", "MAGAZINE_AT_ERROR");
                break;
            }
        }
        telemetry.addData("magazineLaunchTouchSensor.getState():", hzMagazine.magazineLaunchTouchSensor.isPressed());
        telemetry.addData("magazineCollectTouchSensor.getState():", hzMagazine.magazineCollectTouchSensor.isPressed());

        //********** Intake Debug *******
        //telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepad.getDpad_downPress());
        //telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepad.getDpad_upPress());
        //telemetry.addData("intakeMotor.isBusy()", hzIntake.intakeMotor.isBusy());
        switch (hzIntake.getIntakeState()){
            case RUNNING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_RUNNING");
                break;
            }
            case STOPPED: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_STOPPED");
                break;
            }
            case REVERSING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_REVERSING");
                break;
            }
        }

        //******* Launch Controller Debug ********
        telemetry.addData("hzLaunchController.launchMode : ", hzLaunchController.launchMode);
        telemetry.addData("hzLaunchController.launchReadiness : ", hzLaunchController.launchReadiness);
        telemetry.addData("hzLaunchController.launchActivation : ", hzLaunchController.launchActivation);
        telemetry.addData("hzLaunchController.lcTarget : ", hzLaunchController.lcTarget);
        telemetry.addData("hzLaunchController.lcTargetVector", hzLaunchController.lcTargetVector);
        telemetry.addData("hzLaunchController.distanceFromTarget : ", hzLaunchController.distanceFromTarget);
        telemetry.addData("hzLaunchController.lclaunchMotorPower : ", hzLaunchController.lclaunchMotorPower);
        telemetry.addData("hzDrive.drivePointToAlign : ", hzDrive.drivePointToAlign);

        //******* Launcher Debug *********
        //telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition() : ", hzLauncher.launcherRingPlungerServo.getPosition());

        switch (hzLauncher.getLauncherState()){
            case RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_TARGET");
                break;
            }
            case RUNNING_FOR_SUPPLY:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case STOPPED:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_STOPPED");
                break;
            }
        }

        switch (hzArm.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        switch (hzArm.getCurrentArmPosition()){
            case PARKED: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PARKED");
                break;
            }
            case DROP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "DROP_WOBBLE_RING");
                break;
            }
            case HOLD_UP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "HOLD_UP_WOBBLE_RING");
                break;
            }
            case PICK_WOBBLE:{
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_WOBBLE");
                break;
            }
            case PICK_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_RING");
                break;
            }
        }

        telemetry.update();

    }
}

