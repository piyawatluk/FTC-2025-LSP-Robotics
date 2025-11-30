package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Robot_Hardware;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.InputStream;
import java.util.Properties;

/**
 * Utility class for Sequencer-based actions.
 *
 * HOW TO USE:
 * 1) Create new generalUtil(hw)
 * 2) Call util.init(hardwareMap, telemetry)
 * 3) In loop(): call util.Belt(startButton) or util.servo_test(...)
 */
public class generalUtil {

    private final Robot_Hardware hardware;

    // ----------------------------------------
    // Sequencers (NOT STATIC — resets every OpMode run)
    // ----------------------------------------
    private Sequencer sequence1 = new Sequencer();
    private Sequencer lift_seq = new Sequencer();
    private Sequencer belt = new Sequencer();
    private Sequencer shooter = new Sequencer();
    private Sequencer feeder = new Sequencer();

    // Hardware references
    private Servo servo1;
    private DcMotor leftBeltDriveMotor;
    private DcMotor rightBeltDriveMotor;

    // ----------------------------------------
    // Constructor
    // ----------------------------------------
    public generalUtil(Robot_Hardware hw) {
        this.hardware = hw;
    }

    // ----------------------------------------
    // Initialization
    // ----------------------------------------
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Load configs from assets (optional)
        Properties prop = new Properties();
        try (InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")) {
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null)
                telemetry.addData("ERROR", "Cannot read assets/Robot.config");
        }

        // Assign hardware AFTER config
        this.leftBeltDriveMotor = hardware.leftBeltDriveMotor;
        this.rightBeltDriveMotor = hardware.rightBeltDriveMotor;

        //this.servo1 = hardware.servo1; // optional if needed
    }

    // ----------------------------------------
    // SERVO TEST SEQUENCE
    // ----------------------------------------
    //public void servo_test(HardwareMap hardwareMap, boolean start, Telemetry telemetry) {

        //Servo s = hardwareMap.get(Servo.class, "tbd_0");

        //if (s == null) {
            //telemetry.addData("ERROR", "The servo is NULL");
            //return;
        //}

        //if (start) {
            //sequence1 = new Sequencer();  // reset sequence
            //sequence1.add(s, 0.5, 500);           // move to 0.5
            //sequence1.add(s, 0.2, 500, true);     // interpolate to 0.2
            //sequence1.add(830);                   // delay
        //}

        //// Step every loop
        //sequence1.step();
    //}

    public void shooter(boolean bool, double target_RPM){
        double power = target_RPM/6000;
        if (bool){
            hardware.leftShooterMotor.setPower(power);
            hardware.rightShooterMotor.setPower(power);
        }
        else {
            hardware.leftShooterMotor.setPower(0);
            hardware.rightShooterMotor.setPower(0);
        }
    }
    public void feeder(boolean bool){
        if (bool){
            hardware.placeholderServo1.setPower(-1);
            hardware.placeholderServo2.setPower(1);
        }
        else {
            hardware.placeholderServo1.setPower(0);
            hardware.placeholderServo2.setPower(0);
        }
    }

    public void lift(boolean bool, Telemetry telemetry){
        if (bool){
             hardware.liftMotor.setTargetPosition(555);
        }
        else {
            hardware.liftMotor.setTargetPosition(0);
        }
        hardware.liftMotor.setPower(0.5);
        hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("lift position", hardware.liftMotor.getCurrentPosition());
    }
    public void the_gettho(double l2, double l1){
        hardware.rightBeltDriveMotor.setPower(-l2+l1);
    }

    // ----------------------------------------
    // BELT MOTOR SEQUENCE
    // ----------------------------------------
    //public void Belt(boolean start, Telemetry telemetry) {

        //if (leftBeltDriveMotor == null || rightBeltDriveMotor == null)
            //telemetry.addLine("kuy");
            //return;

        //if (start) {
            //leftBeltDriveMotor.setPower(1);
            //rightBeltDriveMotor.setPower(1);

        //}

        //// Step every loop
        //belt.step();
    //}

    // ----------------------------------------
    // Utility: check if sequence finished
    // ----------------------------------------
    //public boolean beltFinished() {
        //return belt.sequenceFinished();
    //}

    //public boolean servoSeqFinished() {
        //return sequence1.sequenceFinished();
    //}
}
