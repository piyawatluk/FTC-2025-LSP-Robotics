package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot_Hardware;


public class generalUtil {

    private final Robot_Hardware hardware;

/*    // (Sequencers kept if you use them elsewhere)
    private Sequencer sequence1 = new Sequencer();
    private Sequencer lift_seq = new Sequencer();
    private Sequencer belt = new Sequencer();
    private Sequencer shooter = new Sequencer();
    private Sequencer feeder = new Sequencer();*/

    // PID state
    private double aimIntegral = 0.0;
    private double prevError = 0.0;
    private long lastTime = 0;

    public generalUtil(Robot_Hardware hw) {
        this.hardware = hw;
    }

    /*public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        Properties prop = new Properties();
        try (InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")) {
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null) telemetry.addData("ERROR", "Cannot read assets/Robot.config");
        }

        // Optional: early validation (uncomment if you prefer fail-fast)
        // validateHardware(telemetry);
    }*/





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

    private void safeSetServoPower(CRServo servo, double power, String nameIfKnown) {
        if (servo == null) {
            RobotLog.w("generalUtil", "Attempt to set power on null servo: %s", nameIfKnown == null ? "<unknown>" : nameIfKnown);
            return;
        }
        try {
            servo.setPower(power);
        } catch (Exception e) {
            RobotLog.ee("generalUtil", e, "Failed to set power on servo: %s", nameIfKnown == null ? "<unknown>" : nameIfKnown);
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
    public void shooter(boolean enabled, double targetRPM) {
        double power = Math.max(0, Math.min(1, targetRPM / 6000.0));
        if (enabled) {
            safeSetMotorPower(hardware.leftShooterMotor, power, "leftShooterMotor");
            safeSetMotorPower(hardware.rightShooterMotor, power, "rightShooterMotor");
            hardware.placeholderServo2.setPosition(1);
            //hardware.placeholderServo2.setPosition(1);
                //safeSetMotorPower(hardware.leftShooterMotor, power, "leftShooterMotor");
                //safeSetMotorPower(hardware.rightShooterMotor, power, "rightShooterMotor");
            //feeder(true);


        }
        else {
            //hardware.placeholderServo2.setPosition(0.5);
            safeSetMotorPower(hardware.leftShooterMotor, 0.0, "leftShooterMotor");
            safeSetMotorPower(hardware.rightShooterMotor, 0.0, "rightShooterMotor");
            hardware.placeholderServo2.setPosition(0.4);
            //feeder(false);
        }

        //double lsm_speed = (new MotorSpeed(hardware.leftShooterMotor).getTicksPerSecond());
        //double rsm_speed = (new MotorSpeed(hardware.rightShooterMotor).getTicksPerSecond());
        //telemetry.addData("lsm speed", lsm_speed);
        //telemetry.addData("rsm speed", rsm_speed);
        //telemetry.addData("delta", lsm_speed-rsm_speed);
    }

    public void gate(boolean atr){
        if (atr){
            hardware.placeholderServo2.setPosition(1);
        } else {
            hardware.placeholderServo2.setPosition(0.3);
        }

    }

    /*public class MotorSpeed {
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
    }*/






    // Teleop feeder helper
    public void feeder(boolean enabled) {
        double p1 = enabled ? -1 : 0;
        double p2 = enabled ? 1 : 0;
        //double pos = enabled ? 0.5:0;
        safeSetServoPower(hardware.placeholderServo1, p1, "placeholderServo1");

        safeSetMotorPower(hardware.rightBeltDriveMotor, p2, "rightBeltDriveMotor");
    }

    public void lift(Telemetry telemetry, int dPadCount) {
        if (hardware.liftMotor == null) {
            RobotLog.w("generalUtil", "liftMotor is null, cannot move lift");
            if (telemetry != null) telemetry.addData("lift", "MISSING liftMotor");
            return;
        }
        try {

            if (dPadCount == 0){
                hardware.liftMotor.setTargetPosition(0);
                telemetry.addLine("lift are at closing position");
            }
            if (dPadCount == 1){
                hardware.liftMotor.setTargetPosition(800);
                telemetry.addLine("lift are at position 1");
            }
            if (dPadCount == 2){
                hardware.liftMotor.setTargetPosition(1300);
                telemetry.addLine("lift are at position 2");
            }

            hardware.liftMotor.setPower(0.6);
            hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (telemetry != null) {
                telemetry.addData("lift position", hardware.liftMotor.getCurrentPosition());

            }
        } catch (Exception e) {
            RobotLog.ee("generalUtil", e, "Error operating liftMotor");
            if (telemetry != null) telemetry.addData("lift", "exception");
        }
    }

    public double Auto_aim(boolean atr, double bearing, Telemetry telemetry) {
        final double Kp = 0.025;
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