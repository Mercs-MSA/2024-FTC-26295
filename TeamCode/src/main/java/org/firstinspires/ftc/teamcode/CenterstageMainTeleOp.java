package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SubSystemDrivetrain;

@TeleOp
//@Config
//@Disabled
//adding empty Functions - update for tele ops
public class CenterstageMainTeleOp extends LinearOpMode {
    private static final double TURN_SPEED = 0.2;
    private static final double DRIVE_SPEED = 0.4;
    public static double SPEED_MULTIPLIER = 1.2;
    private ElapsedTime pauseTimer = new ElapsedTime();
    private boolean hangLiftHang = false;
    private int intakeLiftPosition = 0;
    private boolean clawClosed = false;
    private boolean togglePressed;
    private boolean lastTogglePressed;
    private boolean hangLiftDrop;
    private boolean okayDoEndGame = !SubSystemVariables.protectEndgame;

    private IMU imu;
    //    private SubSystemHopper hopper;
    //   private SubSystemHopperLift hopperLift;
//    private boolean doAutoDropPixel = false;
//    private double targetHeading = 0.0;
//    private double pauseTimerDelay = 0;
//    private boolean hopperOpenManual = false;
//    private boolean hopperOpenAuto = false;
//   private boolean hopperExtendManual = false;
//    private boolean hopperExtendAuto = false;
//    private int hopperLiftPosition;
//    private double hopperLiftSpeed = 1;
//    private boolean hopperLiftDown;
//    private boolean hopperLiftUp;
    private int deltaMultiplier = 1;

//    private enum BACKDROP_ASSIST_STATE {ASSIST_WAIT, ASSIST_ROTATE, ASSIST_APPROACH, ASSIST_RAISE, ASSIST_DROP, ASSIST_RETRACT, ASSIST_PAUSE, HOPPER_OUT, ASSIST_DONE}
//    private BACKDROP_ASSIST_STATE backdropAssistState = BACKDROP_ASSIST_STATE.ASSIST_WAIT;
//    private BACKDROP_ASSIST_STATE backdropAssistStateReturn = BACKDROP_ASSIST_STATE.ASSIST_WAIT;


        private enum ALLIANCE_COLOR {RED, BLUE}
        private enum START_POSITION {LEFT, RIGHT}
        private ALLIANCE_COLOR alianceColor = ALLIANCE_COLOR.RED;
        private START_POSITION startPosition = START_POSITION.LEFT;

        private boolean FieldCentric = true;
        private boolean lastButtonState = false;
        private boolean driveModeChangeButton = false;

        private SubSystemDrivetrain drivetrain = null;

        private double joystickTranslateX = 0.0;
        private double joystickTranslateY = 0.0;
        private double joystickRotate = 0.0;

//    private boolean hangRelease = false;
//    private double hangLiftControl = 0.0;
//    private int intakeDirection = 0;

        private double joystick1LeftXOffset = 0.0;
        private double joystick1LeftYOffset = 0.0;
        private double joystick1RightXOffset = 0.0;
        private double joystick1RightYOffset = 0.0;
        private double joystick2LeftXOffset = 0.0;
        private double joystick2LeftYOffset = 0.0;
        private double joystick2RightXOffset = 0.0;
        private double joystick2RightYOffset = 0.0;
        private ElapsedTime runtime = new ElapsedTime();

        private int liftpos;

    @Override
    public void runOpMode() throws InterruptedException  {
        waitForStart();
        runtime.reset();
        initHardware();
        while (opModeIsActive()) {

        }

    }
        public void initHardware () throws InterruptedException {
            drivetrain = new SubSystemDrivetrain(hardwareMap, SubSystemVariables.currentBot);
            drivetrain.resetGyro();

            FieldCentric = true;
            lastButtonState = false;
            driveModeChangeButton = false;

            gamepadsReset();
        }
        private void waitStart () {
            // Wait for the game to start (driver presses PLAY)
            //Use this time to run the vision code to detect team token position
            // Abort this loop if started or stopped.
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            while (!(isStarted() || isStopRequested())) {
                if (gamepad1.right_bumper && gamepad1.right_trigger > 0.5 && gamepad1.start && gamepad2.right_bumper && gamepad2.right_trigger > 0.5 && gamepad2.start) {
                    drivetrain.resetGyro();
                }
                idle();
            }
        }

        /**
         * Disable all hardware
         */
        private void disableHardware () {

            drivetrain.disableDrivetrainMotors();
            //intakeLift.setIntakeLiftPower(SubSystemVariables.INTAKE_LIFT_POWER);

        }

        private void gamepadsReset ()
        {
            //Measure 'at rest' joystick positions
            joystick1LeftXOffset = 0;//gamepad1.left_stick_x;
            joystick1RightXOffset = 0;//gamepad1.right_stick_x;

            joystick1LeftYOffset = 0;//gamepad1.left_stick_y;
            joystick1RightYOffset = 0;//gamepad1.right_stick_y;


            joystick2LeftXOffset = 0;//gamepad2.left_stick_x;
            joystick2RightXOffset = 0;//gamepad2.right_stick_x;

            joystick2LeftYOffset = 0;//gamepad2.left_stick_y;
            joystick2RightYOffset = 0;//gamepad2.right_stick_y;
        }

        private void gamepadsUpdate ()
        {
            //Drive base motion controls
            joystickTranslateX = gamepad1.left_stick_x - joystick1LeftXOffset;
            joystickTranslateY = gamepad1.left_stick_y - joystick1LeftYOffset;
            joystickRotate = gamepad1.right_stick_x - joystick1RightXOffset;

        }


        private void drivebaseUpdate ()
        {
            double translateSpeed = Math.hypot(joystickTranslateX, joystickTranslateY) * SPEED_MULTIPLIER;
            //Heading 0 = forward, -ve right, +ve left
            double heading = Math.atan2(-joystickTranslateX, -joystickTranslateY);

            drivetrain.doMecanumDrive(translateSpeed, heading, joystickRotate, FieldCentric);
        }

        private void telemetryUpdate () {

            telemetry.addData("Timer: ", pauseTimer.seconds());
            telemetry.addData("front left power ", SubSystemDrivetrain.FLP);
            telemetry.addData("front right power ", SubSystemDrivetrain.FRP);
            telemetry.addData("back left power ", SubSystemDrivetrain.BLP);
            telemetry.addData("back right power ", SubSystemDrivetrain.BRP);
        }

        private boolean robotIsFacingBackdrop ( double margin){
            //Is the robot facing the backdrop (ish)?
            //Within the specified error angle of the backdrop?
            //Make read and blue the same. Technically this would also allow the bot to face away
            //from the backdrop, but good enough for the moment
            double currentAbsHeading = Math.abs(drivetrain.getCurrentHeading(false));//Result in degrees
            //Robot heading is 90 degrees rotated to the 'field' 0 heading
            if (Math.abs((currentAbsHeading - 90)) < margin)
                return true;
            else
                return false;
        }


        /**
         * This is the main op mode and should call all the initialization, wait for start,
         * execute your desired auto/tele-op, then stop everything
         */
    }

