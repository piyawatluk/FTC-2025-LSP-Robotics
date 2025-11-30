package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    }

    // Simple teleop shooter helper
    public void shooter(boolean enabled, double targetRPM) {
        double power = Math.max(0, Math.min(1, targetRPM / 6000.0));
        if (enabled) {
            hardware.leftShooterMotor.setPower(power);
            hardware.rightShooterMotor.setPower(power);
        } else {
            hardware.leftShooterMotor.setPower(0);
            hardware.rightShooterMotor.setPower(0);
        }
    }

    // Standalone timed shooter action (spins, then stops at end)
    public Action shooterAction(double targetRPM, double durationSeconds) {
        final double rpm = Math.max(0, targetRPM);
        final double dur = Math.max(0, durationSeconds);

        return new Action() {
            private long startNs = 0;
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) { started = true; startNs = System.nanoTime(); }

                double elapsed = (System.nanoTime() - startNs) / 1e9;
                double power = Math.max(0, Math.min(1, rpm / 6000.0));

                hardware.leftShooterMotor.setPower(power);
                hardware.rightShooterMotor.setPower(power);

                if (packet != null) {
                    packet.put("ShooterAction RPM", rpm);
                    packet.put("ShooterAction Elapsed", elapsed);
                }

                if (elapsed >= dur) {
                    hardware.leftShooterMotor.setPower(0);
                    hardware.rightShooterMotor.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    // Composite: spin up, feed while holding RPM, then stop
    public Action fireAction(double rpm, double spinUpSeconds, double feedSeconds) {
        final double R = Math.max(0, rpm);
        final double T1 = Math.max(0, spinUpSeconds);
        final double T2 = Math.max(0, feedSeconds);

        return new SequentialAction(
                shooterHold(R, T1, /*stopWhenDone=*/false),
                new ParallelAction(
                        feedFor(T2),
                        shooterHold(R, T2, /*stopWhenDone=*/false)
                ),
                shooterHold(R, 0.0, /*stopWhenDone=*/true) // quick stop
        );
    }

    // Keep shooter powered for 'seconds'; stop at end if requested
    private Action shooterHold(double rpm, double seconds, boolean stopWhenDone) {
        final double R = Math.max(0, rpm);
        final double S = Math.max(0, seconds);

        return new Action() {
            private long startNs = 0;
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) { started = true; startNs = System.nanoTime(); }
                double elapsed = (System.nanoTime() - startNs) / 1e9;

                double power = Math.max(0, Math.min(1, R / 6000.0));
                hardware.leftShooterMotor.setPower(power);
                hardware.rightShooterMotor.setPower(power);

                if (packet != null) {
                    packet.put("ShooterHold RPM", R);
                    packet.put("ShooterHold Elapsed", elapsed);
                }

                if (elapsed >= S) {
                    if (stopWhenDone) {
                        hardware.leftShooterMotor.setPower(0);
                        hardware.rightShooterMotor.setPower(0);
                    }
                    return false;
                }
                return true;
            }
        };
    }

    // Run feeder for 'seconds' (expects CRServo; otherwise change types)
    private Action feedFor(double seconds) {
        final double S = Math.max(0, seconds);

        return new Action() {
            private long startNs = 0;
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) { started = true; startNs = System.nanoTime(); }
                double elapsed = (System.nanoTime() - startNs) / 1e9;

                hardware.placeholderServo1.setPower(-1);
                hardware.placeholderServo2.setPower(1);

                if (packet != null) {
                    packet.put("Feed Elapsed", elapsed);
                }

                if (elapsed >= S) {
                    hardware.placeholderServo1.setPower(0);
                    hardware.placeholderServo2.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    // Teleop feeder helper
    public void feeder(boolean enabled) {
        double p1 = enabled ? -1 : 0;
        double p2 = enabled ? 1 : 0;
        hardware.placeholderServo1.setPower(p1);
        hardware.placeholderServo2.setPower(p2);
    }

    public Action motorAction(DcMotor motor, double power, double durationSeconds) {
        final double p = Math.max(-1, Math.min(1, power));
        final double dur = Math.max(0, durationSeconds);

        return new Action() {
            private long startNs = 0;
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    startNs = System.nanoTime();
                }

                double elapsed = (System.nanoTime() - startNs) / 1e9;

                // apply power
                motor.setPower(p);

                if (packet != null) {
                    packet.put("MotorAction Power", p);
                    packet.put("MotorAction Elapsed", elapsed);
                }

                // stop after time is up
                if (elapsed >= dur) {
                    motor.setPower(0);
                    return false; // done
                }

                return true; // keep running
            }
        };
    }

    public void lift(boolean up, Telemetry telemetry) {
        hardware.liftMotor.setTargetPosition(up ? 555 : 0);
        hardware.liftMotor.setPower(0.2);
        hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (telemetry != null) telemetry.addData("lift position", hardware.liftMotor.getCurrentPosition());
    }

    public void Exterior_Feeder(double l2, double l1) {
        hardware.rightBeltDriveMotor.setPower(-l2 + l1);
    }

    public double Auto_aim(boolean atr, double bearing, double current_heading, Telemetry telemetry) {
        final double Kp = 0.25;
        final double Ki = 0.001;
        final double Kd = 0.0;

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
