package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Robot_Hardware;

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

    private double aimIntegral = 0.0;
    private double prevError = 0.0;
    private long lastTime = 0;

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

        // Load configs from assets
        Properties prop = new Properties();
        try (InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")) {
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null)
                telemetry.addData("ERROR", "Cannot read assets/Robot.config");
        }
    }

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
        hardware.liftMotor.setPower(0.2); //subject to change
        hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("lift position", hardware.liftMotor.getCurrentPosition());
    }
    public void Exterior_Feeder(double l2, double l1){
        hardware.rightBeltDriveMotor.setPower(-l2+l1);
    }

    public double Auto_aim(boolean atr, double bearing, double current_heading, Telemetry telemetry) {

        // PID gains — tune these on the robot
        final double Kp = 0.25;
        final double Ki = 0.001;     // start at 0, raise slowly
        final double Kd = 0.0;

        long now = System.nanoTime();
        double dt = 0.0;

        if (lastTime != 0) {
            dt = (now - lastTime) / 1e9;   // seconds
        }
        lastTime = now;

        double error = bearing;

        // --- Integral term ---
        if (Double.isFinite(dt) && dt > 0) {
            aimIntegral += error * dt;

            // prevent runaway
            double maxIntegral = 2.0;
            if (aimIntegral > maxIntegral) aimIntegral = maxIntegral;
            if (aimIntegral < -maxIntegral) aimIntegral = -maxIntegral;
        }

        // --- Derivative term ---
        double derivative = 0.0;
        if (Double.isFinite(dt) && dt > 0) {
            derivative = (error - prevError) / dt;
        }
        prevError = error;

        // --- PID output ---
        double respond_val = -(Kp * error + Ki * aimIntegral + Kd * derivative);

        if (!Double.isFinite(respond_val)) respond_val = 0.0;

        // clamp drive command
        double output = Math.max(-1.0, Math.min(1.0, respond_val));

        if (!atr) return 0.0;

        // small deadband for “aligned”
        if (Math.abs(error) < 1.0) {
            telemetry.addLine("Aligned");
            return 0.0;
        }

        if (output < 0) {
            telemetry.addLine("Turn right");
        } else if (output > 0) {
            telemetry.addLine("Turn left");
        }

        return output;
    }
}
