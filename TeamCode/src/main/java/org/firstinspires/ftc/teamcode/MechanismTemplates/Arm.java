package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    private  PIDFController armPIDF;
    private Motor armMotor;

    // Declaring and Initializing PIDF values
    public static double armKp = 0.0026;
    public static double armKi = 0.000029;
    public static double armKd = 0.000005;
    public static double armKf = 0.000004;

    public static double EXTAKE_POS = 1300; // 1255 Actual position based on encoder readings
    public static double INTAKE_POS = 0; // -42.45391238 was the old value

    // Initially set to 0 because we only want the claw to move when given input from the controller
    // initializing the targetPos value to a greater positive value would cause the update() method to
    // immediately start moving the arm since a difference between the current motor encoder position
    // and the target position is created (error).
    public double targetPos = 0.0;

    public Arm(HardwareMap hardwareMap){
        armMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_60); // Pin ___
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.VelocityControl);

        armMotor.resetEncoder(); // We want the arm to start at a position of 0

        double[] PIDF_COEFF = {armKp, armKi, armKd, armKf};
        armPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2], PIDF_COEFF[3]);
    }

    public void update(Telemetry telemetry){
        armPIDF.setPIDF(armKp, armKi, armKd, armKf);
        // Correction represents the error term of the PIDF loop
        double correction = armPIDF.calculate(armMotor.getCurrentPosition(), targetPos);

       // telemetry.addData("Correction: ", correction);
        //telemetry.addData("Target Position: ", targetPos);
       // telemetry.addData("Motor Position: ", armMotor.getCurrentPosition());
       // telemetry.update();

        armMotor.set(correction);
    }

    public void setExtake(){
        targetPos = EXTAKE_POS;
    }

    public void setIntake(){
        targetPos = INTAKE_POS;
    }
}
