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
        actionQueue.add(new ServoAction(servo, targetPos, durationMs));
    }

    // degree version of addAction
    public void addAction(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg) {
        double targetPos = degToPos(targetPos_deg, maxDeg, minDeg);
        addAction(servo, clamp(targetPos), durationMs);  // clamp makes sure the value isn't illegal
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

        // interpolated movement
        double t = clamp(timer.milliseconds() / currentAction.durationMs);
        double interpolatedPos = currentAction.startPos + (currentAction.targetPos - currentAction.startPos) * t;
        safeSetPosition(currentAction.servo, interpolatedPos);
        if (t >= 1.0) {
            currentAction = null; // Action is finished
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
        public Servo servo;
        public double targetPos;
        public long durationMs;
        public double startPos;

        public ServoAction(Servo servo, double targetPos, long durationMs) {
            this.servo = servo;
            this.targetPos = targetPos;
            this.durationMs = durationMs;
        }
    }
}