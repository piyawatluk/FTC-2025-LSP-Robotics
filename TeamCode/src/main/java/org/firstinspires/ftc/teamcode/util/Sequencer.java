package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class Sequencer {
    ElapsedTime timer = new ElapsedTime();
    Queue<SeqAction> actionQueue = new LinkedList<>();
    SeqAction currentAction = null;

    public void add(Servo servo, double targetPos, long durationMs) {
        actionQueue.add(new ServoAction(servo, ServoUtil.clamp(targetPos), durationMs));
    }

    public void add(Servo servo, double targetPos, long durationMs, boolean interpolate) {
        actionQueue.add(new ServoAction(servo, ServoUtil.clamp(targetPos), durationMs, interpolate));
    }

    public void add(DcMotor motor, double power, long durationMs) {
        actionQueue.add(new MotorAction(motor, power, durationMs));
    }

    public void add(long durationMs) {
        actionQueue.add(new Delay(durationMs));
    }

    // degree version of addAction
    public void add(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg) {
        double targetPos = ServoUtil.degToPos(targetPos_deg, maxDeg, minDeg);
        add(servo, ServoUtil.clamp(targetPos), durationMs);
    }

    public void add(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg, boolean interpolate) {
        double targetPos = ServoUtil.degToPos(targetPos_deg, maxDeg, minDeg);
        add(servo, ServoUtil.clamp(targetPos), durationMs, interpolate);
    }


    public void step() {
        if (currentAction == null) {
            currentAction = actionQueue.poll();
            if (currentAction == null) return;
            if (currentAction instanceof ServoAction && ((ServoAction) currentAction).interpolate) {
                ((ServoAction) currentAction).startPos = ((ServoAction) currentAction).servo.getPosition();
            }

            timer.reset();
        }

        if (currentAction.execute()) currentAction = null;
    }

    public boolean sequenceFinished () {
        return currentAction == null && actionQueue.isEmpty();
    }

    public void clearSeq () {
        actionQueue.clear();
        currentAction = null;
    }

    public interface SeqAction {
        boolean execute();
    }

    public final class ServoAction implements SeqAction {
        public final Servo servo;
        public final double targetPos;
        public final long durationMs;
        public double startPos;
        public final boolean interpolate;

        public ServoAction(Servo servo, double targetPos, long durationMs) {
            this.servo = servo;
            this.targetPos = targetPos;
            this.durationMs = durationMs;
            this.interpolate = false;
        }

        public ServoAction(Servo servo, double targetPos, long durationMs, boolean interpolate) {
            this.servo = servo;
            this.targetPos = targetPos;
            this.durationMs = durationMs;
            this.interpolate = interpolate;
        }

        @Override
        public boolean execute() {
            if (interpolate && durationMs > 0) {
                // interpolated movement
                double t = Math.min(1.0, timer.milliseconds() / durationMs);
                double interpolatedPos = startPos + (targetPos - startPos) * t;
                ServoUtil.safeSetPosition(servo, interpolatedPos);
                return t >= 1.0;
            } else {
                ServoUtil.safeSetPosition(servo, targetPos);
                return timer.milliseconds() >= durationMs;
            }
        }
    }

    public final class MotorAction implements SeqAction {
        public final DcMotor motor;
        public final double power;
        public final long durationMs;

        public MotorAction(DcMotor motor, double power, long durationMs) {
            this.motor = motor;
            this.power = power;
            this.durationMs = durationMs;
        }

        @Override
        public boolean execute() {
            motor.setPower(Math.min(1, Math.max(-1, power)));
            if (timer.milliseconds() >= durationMs) {
                motor.setPower(0);
                return true;
            }
            return false;
        }
    }

    public final class Delay implements SeqAction {
        public final long durationMs;

        public Delay(long durationMs) {
            this.durationMs = durationMs;
        }

        @Override
        public boolean execute() {
            return timer.milliseconds() >= durationMs;
        }
    }
}