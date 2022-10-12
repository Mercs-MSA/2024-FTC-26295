/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopDriving", group="Concept")
//@Disabled
public class TeleopDriving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;

    // slider motor variables
    private DcMotor SliderMotor = null;
    static final int FOUR_STAGE_SLIDER_MAX_POS = 4200 - 100;  // Leave 100 counts for buffer.
    static final int SLIDER_MIN_POS = 0;
    static final int POSITION_COUNTS_FOR_ONE_REVOLUTION = 512; // Approximate value from testing
    static final int LOW_JUNCTION_POS = 1300; // need double check by testing
    static final int MEDIUM_JUNCTION_POS = 2600;
    static final int HIGH_JUNCTION_POS = 3900;

    int sliderMotorTargetPosition = 0;
    int motorPositionInc = POSITION_COUNTS_FOR_ONE_REVOLUTION/4; // will update this value based on testing

    // claw servo motor variables
    private Servo clawServo = null;
    static final double CLAW_INCREMENT = 0.002;     // amount to slew servo each CYCLE_MS cycle
    static final double CLAW_MAX_POS = 0.3;     // Maximum rotational position
    static final double CLAW_MIN_POS = 0.08;     // Minimum rotational position
    double clawServoPosition = CLAW_MAX_POS;


    // arm servo variables
    static final double ARM_INCREMENT = 0.002;     // amount to slew servo each CYCLE_MS cycle
    static final double ARM_MAX_POS = 0.3;     // Maximum rotational position
    static final double ARM_MIN_POS = 0.08;     // Minimum rotational position
    private Servo armServo = null;
    double armServoPosition = ARM_MAX_POS;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class,"BackRight");
        SliderMotor = hardwareMap.get(DcMotor.class,"SliderMotor");
        armServo = hardwareMap.get(Servo.class, "ArmServo");
        clawServo = hardwareMap.get(Servo.class, "TestServo");

        // claw servo motor initial
        clawServoPosition = CLAW_INCREMENT;
        clawServo.setPosition(clawServoPosition);
        telemetry.addData("Status", "claw Servo init position %.2f", clawServoPosition);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // slider motor control
        SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE); // it based on how Motor installed on robot.
        SliderMotor.setTargetPosition(sliderMotorTargetPosition);
        // Reset slider motor encoder counts kept by the motor
        SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Using encoder mode to run slider motor
        SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor to run to target encoder position and top with brakes on.
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontLeftPower;
            double FrontRightPower;
            double BackLeftPower;
            double BackRightPower;
            double SliderMotorPower = 0.7; // apply 70% of maximum speed to move the slider

            double drive = 0.7*(gamepad1.left_stick_y);
            double turn  =  0.6 * (-gamepad1.right_stick_x);
            double strafe = 0.7*(-gamepad1.left_stick_x);

            FrontLeftPower    = Range.clip(-drive - turn - strafe, -1, 1) ;
            FrontRightPower   = Range.clip(-drive + turn + strafe, -1, 1) ;
            BackLeftPower    = Range.clip(-drive - turn + strafe, -1, 1) ;
            BackRightPower   = Range.clip(-drive + turn - strafe, -1, 1) ;
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            FrontLeftDrive.setPower(FrontLeftPower);
            FrontRightDrive.setPower(FrontRightPower);
            BackLeftDrive.setPower(BackLeftPower);
            BackRightDrive.setPower(BackRightPower);


            telemetry.addData("Status", "slider motor current position %d", SliderMotor.getCurrentPosition());
            telemetry.update();

            // use Y button to lift up the slider reaching high junction
            if (gamepad1.y)
            {
                sliderMotorTargetPosition = HIGH_JUNCTION_POS;
            }

            // use B button to lift up the slider reaching medium junction
            if (gamepad1.b)
            {
                sliderMotorTargetPosition = MEDIUM_JUNCTION_POS;
            }

            // use A button to lift up the slider reaching low junction
            if (gamepad1.a)
            {
                sliderMotorTargetPosition = LOW_JUNCTION_POS;
            }

            // use X button to move the slider back to lowest position (ground junction)
            if (gamepad1.x)
            {
                sliderMotorTargetPosition = 0;
            }

            // use right stick_Y to lift or down slider continuously
            sliderMotorTargetPosition -= (gamepad1.right_stick_y) * motorPositionInc;
            if (sliderMotorTargetPosition > FOUR_STAGE_SLIDER_MAX_POS)
            {
                sliderMotorTargetPosition = FOUR_STAGE_SLIDER_MAX_POS;
            }
            if (sliderMotorTargetPosition < SLIDER_MIN_POS)
            {
                sliderMotorTargetPosition = SLIDER_MIN_POS;
            }

            telemetry.addData("Status", "slider motor Target position %d", sliderMotorTargetPosition);

            SliderMotor.setTargetPosition(sliderMotorTargetPosition);
            SliderMotor.setPower(SliderMotorPower); // slider motor start movement

            /* for debugging
            resetRuntime();
            while (SliderMotor.isBusy() && (getRuntime()< 3))  // wait while slider motor is busy running to position
            {
                telemetry.addData("Status", "slider motor current position %d", SliderMotor.getCurrentPosition());
                telemetry.update();
                //idle();
            }
            */

            telemetry.addData("Status", "slider motor current position %d", SliderMotor.getCurrentPosition());
            telemetry.update();

            //SliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // turn off motor by set power to zero.
            // The motor stop on their own but power is still applied.
            //SliderMotor.setPower(0.0);

            if (gamepad1.dpad_up) {
                // Keep stepping up until we hit the max value.
                clawServoPosition += CLAW_INCREMENT;
                if (clawServoPosition >= CLAW_MAX_POS) {
                    clawServoPosition = CLAW_MAX_POS;
                }
            }
            else if (gamepad1.dpad_down) {
                clawServoPosition -= CLAW_INCREMENT;
                if (clawServoPosition <= CLAW_MIN_POS) {
                    clawServoPosition = CLAW_MIN_POS;
                }
            }
            clawServo.setPosition(clawServoPosition);
            telemetry.addData("Status", "Servo position %.2f", clawServoPosition);

            // arm servo motor control
            if (gamepad1.dpad_left) {
                // Keep stepping up until we hit the max value.
                armServoPosition += ARM_INCREMENT;
                if (armServoPosition >= ARM_MAX_POS) {
                    armServoPosition = ARM_MAX_POS;
                }
            }
            else if (gamepad1.dpad_right) {
                armServoPosition -= ARM_INCREMENT;
                if (armServoPosition <= ARM_MIN_POS) {
                    armServoPosition = ARM_MIN_POS;
                }
            }
            armServo.setPosition(armServoPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f)", FrontLeftPower, FrontRightPower, BackLeftPower,BackRightPower);
            telemetry.update();
        }
    }
}
