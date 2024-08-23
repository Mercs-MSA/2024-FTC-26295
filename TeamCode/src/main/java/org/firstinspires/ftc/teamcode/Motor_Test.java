package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



//This is a Robot Function test Routine to push to github
@TeleOp

//@Disabled
public class Motor_Test extends LinearOpMode {
    //Dashboard demo variables
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    //Motor demo variables
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private int motorToTest = 2;
    //private DcMotorEx hopperLift;
    private float powerToSet;
    private boolean pressedOnce = true;

    public void initializeHardware() throws InterruptedException {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
       // intakeLift = hardwareMap.get(DcMotorEx.class, "intakeLift");
        //hopperLift = hardwareMap.get(DcMotorEx.class, "hopperLift");
    //    intake = hardwareMap.get(DcMotorEx.class, "intake");
     //   hangMotor = hardwareMap.get(DcMotorEx.class, "hangLift");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hopperLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     //   hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //hopperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //    hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hopperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


 //       testServo = hardwareMap.get(Servo.class, "hopperGateServo");
        //torqueServo = hardwareMap.get(Servo.class, "torqueServo");
 //       HOPPER_SERVO_OLD = hardwareMap.get(Servo.class, "hopperServo");
        //hopperServo = new SubSystemHopper(hardwareMap);
        //hangLift = new SubSystemHangLift(hardwareMap);
    }

    private static void initTestmotor() {

    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }


    public void dashboardDemo(){
        double time = getRuntime();

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;

        double[] bxPoints = { l, -l, -l, l };
        double[] byPoints = { l, l, -l, -l };
        rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }


        sleep(20);
    }

    private void displayTelemetry() {
        telemetry.addData("Motor (intake is true, hopper is false)", motorToTest);

 //        telemetry.addData("Motor 3 encoder: ", hangMotor.getCurrentPosition());
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();
        while (opModeIsActive()) {
            displayTelemetry();
            updateController();
            //hangLiftUpdate();
            updateTestMotors();

        }
    }

    private void updateTestMotors() {
        if(motorToTest == 1) {
            CenterstageAutonomousMain();
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if(motorToTest == 2) {
            MecanumDrivingSampleField();
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        if(motorToTest == 3) {
            MecanumDrivingSampleFieldwPID();
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if(motorToTest == 4) {
             leftFrontDrive.setPower(powerToSet);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if(motorToTest == 5) {
           leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(powerToSet);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if(motorToTest == 6) {
           // intakeLift.setPower(0);
 //           hangMotor.setPower(0);
            //hopperLift.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(powerToSet);
            rightBackDrive.setPower(0);
        }

        if(motorToTest == 7) {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(powerToSet);
         }

    }

    private void updateController() {
        /*
        if(gamepad1.left_bumper)
            setServoPos(0.8); //0.8 is open for gate servo

        if(gamepad1.right_bumper)
            setServoPos(1); //1 is closed gate servo

        if(gamepad1.left_trigger > 0.5)
            setTorqueServoPos(0);

        if(gamepad1.right_trigger > 0.5)
            setTorqueServoPos(1);

        if(gamepad1.x)
            setHopperServo(servoMax); //HopperServo is 0.05 for down

        if(gamepad1.b)
            setHopperServo(servoMin); //HopperServo is 0.4 for up
         */

        boolean endgame = true;


        if(gamepad1.dpad_left && pressedOnce) {
            motorToTest++;
            pressedOnce = false;
        } else if (!gamepad1.dpad_left){
            pressedOnce = true;
        }
        if(motorToTest > 7) {
            motorToTest = 1;
        }

        if(gamepad1.left_trigger > 0.1) {
            powerToSet = gamepad1.left_trigger;
            telemetry.addLine("powering motor");
        } else if (gamepad1.right_trigger > 0.1 ) {
            powerToSet = -gamepad1.right_trigger;
            telemetry.addLine("powering motor");
        } else {
            powerToSet = 0;
            telemetry.addLine("Not powering any motor");
        }

        if (gamepad1.right_bumper) {
     //       hopperServo.openGate(true);
        }

        if (gamepad1.left_bumper) {
    //        hopperServo.openGate(false);
        }

        if(gamepad1.x) {
    //        hopperServo.setHopperPosition(SubSystemVariables.HOPPER_POS_1);
        }
        if(gamepad1.y) {
     //       hopperServo.setHopperPosition(SubSystemVariables.HOPPER_POS_3);
        }
        if(gamepad1.b) {
     //       hopperServo.setHopperPosition(SubSystemVariables.HOPPER_POS_4);
        }

    }


 }

