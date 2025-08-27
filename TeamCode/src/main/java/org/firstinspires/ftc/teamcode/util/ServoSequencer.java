package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSequencer extends ServoUtil {
    ElapsedTime timer = new ElapsedTime();
    Queue<ServoAction> actionQueue = new LinkedList<>();
    ServoAction currentAction = null;

    public void addAction(Servo servo, double targetPos, long durationMs) {
        actionQueue.add(new ServoAction(servo, clamp(targetPos), durationMs));
    }

    public void addAction(Servo servo, double targetPos, long durationMs, boolean interpolate) {
        actionQueue.add(new ServoAction(servo, clamp(targetPos), durationMs, interpolate));
    }

    // degree version of addAction
    public void addAction(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg) {
        double targetPos = degToPos(targetPos_deg, maxDeg, minDeg);
        addAction(servo, clamp(targetPos), durationMs);
    }

    public void addAction(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg, boolean interpolate) {
        double targetPos = degToPos(targetPos_deg, maxDeg, minDeg);
        addAction(servo, clamp(targetPos), durationMs, interpolate);
    }

    public void stepSequence() {
        if (currentAction == null) {
            currentAction = actionQueue.poll();
            if (currentAction == null) {
                return;
            }
            try {
                currentAction.startPos = currentAction.servo.getPosition();
            } catch (NullPointerException e) {
                currentAction = null;
                return;
            }
            timer.reset();
        }

        if (currentAction.interpolate && currentAction.durationMs <= 0) {
            // interpolated movement
            double t = Math.min(1.0,timer.milliseconds() / currentAction.durationMs);
            double interpolatedPos = currentAction.startPos + (currentAction.targetPos - currentAction.startPos) * t;
            safeSetPosition(currentAction.servo, interpolatedPos);
            if (t >= 1.0) {
                currentAction = null; // Action is finished
            }
        } else {
            safeSetPosition(currentAction.servo, currentAction.targetPos);
            if (timer.milliseconds() >= currentAction.durationMs) {
                currentAction = null;
            }
        }
    }

    public boolean actionFinished () {
        return currentAction == null && actionQueue.isEmpty();
    }

    public void clearSeq () {
        actionQueue.clear();
        currentAction = null;
    }

    public static final class ServoAction {
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
    }
}