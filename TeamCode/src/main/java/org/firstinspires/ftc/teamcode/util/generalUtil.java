package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

import java.io.InputStream;
import java.util.Properties;

public class generalUtil {

    private final Robot_Hardware hardware;

    // (Sequencers kept if you use them elsewhere)
    private Sequencer sequence1 = new Sequencer();
    private Sequencer lift_seq = new Sequencer();
    private Sequencer belt = new Sequencer();
    private Sequencer shooter = new Sequencer();
    private Sequencer feeder = new Sequencer();
    private Sequencer barrier_Gate = new Sequencer();

    // Barrier gate
    public double gatePos1 = 0.85;
    public double gatePos2 = 1;

    public boolean gateBusy = false;

    // PID state
    private double aimIntegral = 0.0;
    private double prevError = 0.0;
    private long lastTime = 0;

    public generalUtil(Robot_Hardware hw) {
        this.hardware = hw;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        Properties prop = new Properties();
        try (InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")) {
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null) telemetry.addData("ERROR", "Cannot read assets/Robot.config");
        }

        // Optional: early validation (uncomment if you prefer fail-fast)
        // validateHardware(telemetry);
    }
    public void updateSequences() {
        if (gateBusy) {
            barrier_Gate.step();  // runs the current action

            if (barrier_Gate.sequenceFinished()) {
                gateBusy = false; // done, free to shoot again
            }
        }
    }
    public void startGateSequence() {
        gateBusy = true;

        // Clear any old actions
        barrier_Gate.clearSeq();

        // 1) Open gate to gatePos2 and wait 200ms
        barrier_Gate.add(hardware.placeholderServo3, gatePos2, 4000);

        // 2) Close gate to gatePos1, no extra wait needed
        barrier_Gate.add(hardware.placeholderServo3, gatePos1, 0);
    }
    //For april tag freezing

    public boolean isGateBusy() {
        return gateBusy;
    }

    public boolean shouldFreezeAprilTag() {
        return gateBusy;
    }





    // -- Helper safe setters to avoid NPEs --
    private void safeSetMotorPower(DcMotor motor, double power, String nameIfKnown) {
        if (motor == null) {
            RobotLog.w("generalUtil", "Attempt to set power on null motor: %s", nameIfKnown == null ? "<unknown>" : nameIfKnown);
            return;
        }
        try {
            motor.setPower(power);
        } catch (Exception e) {
            RobotLog.ee("generalUtil", e, "Failed to set power on motor: %s", nameIfKnown == null ? "<unknown>" : nameIfKnown);
        }
    }
    private void safeSetServoPosition(Servo servo, double position, String nameIfKnown) {
        if (servo == null) {
            RobotLog.w("generalUtil", "Attempt to set position on null servo: %s",
                    nameIfKnown == null ? "<unknown>" : nameIfKnown);
            return;
        }
        try {
            servo.setPosition(position);
        } catch (Exception e) {
            RobotLog.ee("generalUtil", e, "Failed to set position on servo: %s",
                    nameIfKnown == null ? "<unknown>" : nameIfKnown);
        }
    }

    // Optional helper to validate critical hardware and report via telemetry
    public void validateHardware(Telemetry telemetry) {
        if (hardware == null) {
            RobotLog.e("generalUtil", "Robot_Hardware is null");
            if (telemetry != null) telemetry.addData("HW_ERROR", "Robot_Hardware is null");
            return;
        }

        if (telemetry != null) {
            telemetry.addData("HW leftShooter", hardware.leftShooterMotor == null ? "MISSING" : "OK");
            telemetry.addData("HW rightShooter", hardware.rightShooterMotor == null ? "MISSING" : "OK");
            telemetry.addData("HW liftMotor", hardware.liftMotor == null ? "MISSING" : "OK");
            telemetry.addData("HW placeholderServo1", hardware.placeholderServo1 == null ? "MISSING" : "OK");
            telemetry.addData("HW placeholderServo2", hardware.placeholderServo2 == null ? "MISSING" : "OK");
            telemetry.addData("HW rightBeltDriveMotor", hardware.rightBeltDriveMotor == null ? "MISSING" : "OK");
        }
    }

    // Simple teleop shooter helper
    public void shooter(boolean enabled, double targetRPM, Telemetry telemetry) {
        double power = Math.max(0, Math.min(1, targetRPM / 6000.0));
        double powerR = Math.max(0, Math.min(1, (targetRPM-500) / 6000.0));
        if (enabled) {
            safeSetMotorPower(hardware.leftShooterMotor, power, "leftShooterMotor");
            safeSetMotorPower(hardware.rightShooterMotor, powerR, "rightShooterMotor");
        } else {
            safeSetMotorPower(hardware.leftShooterMotor, 0.0, "leftShooterMotor");
            safeSetMotorPower(hardware.rightShooterMotor, 0.0, "rightShooterMotor");
        }

        double lsm_speed = (new MotorSpeed(hardware.leftShooterMotor).getTicksPerSecond());
        double rsm_speed = (new MotorSpeed(hardware.rightShooterMotor).getTicksPerSecond());
        telemetry.addData("lsm speed", lsm_speed);
        telemetry.addData("rsm speed", rsm_speed);
        telemetry.addData("delta", lsm_speed-rsm_speed);
    }

    public class MotorSpeed {
        private DcMotor motor;
        private ElapsedTime timer = new ElapsedTime();
        private int lastPosition = 0;

        public MotorSpeed(DcMotor motor) {
            this.motor = motor;
            this.lastPosition = motor.getCurrentPosition();
            timer.reset();
        }

        // returns ticks per second (TPS)
        public double getTicksPerSecond() {
            int currentPos = motor.getCurrentPosition();
            double dt = timer.seconds();

            int deltaPos = currentPos - lastPosition;

            // update for next cycle
            lastPosition = currentPos;
            timer.reset();

            return deltaPos / dt;
        }
    }

    public double Auto_aim(boolean atr, double bearing, Telemetry telemetry) {
        final double Kp = 0.03;
        final double Ki = 0.0001;
        final double Kd = 0.0001;

        long now = System.nanoTime();
        double dt = (lastTime != 0) ? (now - lastTime) / 1e9 : 0.0;
        lastTime = now;

        double error = bearing;

        if (dt > 0 && Double.isFinite(dt)) {
            aimIntegral += error * dt;
            double maxI = 2.0;
            if (aimIntegral > maxI) aimIntegral = maxI;
            if (aimIntegral < -maxI) aimIntegral = -maxI;
        }

        double derivative = (dt > 0 && Double.isFinite(dt)) ? (error - prevError) / dt : 0.0;
        prevError = error;

        double respond = -(Kp * error + Ki * aimIntegral + Kd * derivative);
        if (!Double.isFinite(respond)) respond = 0.0;

        double output = Math.max(-1.0, Math.min(1.0, respond));

        if (!atr) return 0.0;

        if (Math.abs(error) < 1.0) {
            if (telemetry != null) telemetry.addLine("Aligned");
            return 0.0;
        }

        if (telemetry != null) {
            telemetry.addLine(output < 0 ? "Turn right" : "Turn left");
        }
        return output;
    }
}