package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * For queueing and executing actions
 *
 * <p>Add actions in start() and call step() repeatedly in loop()</p>
 * <p>Actions can be made by implementing SeqAction Interface and making an add() overload</p>
 */
public class Sequencer {
    ElapsedTime timer = new ElapsedTime();
    Queue<SeqAction> actionQueue = new LinkedList<>();
    SeqAction currentAction = null;
    public int actionCounter = 0;



    /**
     * Adds a Servo Action to Queue
     * @param servo Servo you want to move
     * @param targetPos Target Position [0.0 - 1.0]
     * @param durationMs Time to wait after finishing the action
     * @see #add(Servo, double, long, boolean)
     * @see #add(Servo, int, long, int, int)
     * @see #add(Servo, int, long, int, int, boolean)
     */
    public void add(Servo servo, double targetPos, long durationMs) {
        actionQueue.add(new ServoAction(servo, ServoUtil.clamp(targetPos), durationMs));
    }

    /**
     * Adds a Servo Action with optional interpolation to Queue
     * @param servo Servo you want to move
     * @param targetPos Target Position [0.0 - 1.0]
     * @param durationMs Time to interpolate over
     * @param interpolate Whether to interpolate or not
     * @see #add(Servo, double, long)
     * @see #add(Servo, int, long, int, int)
     * @see #add(Servo, int, long, int, int, boolean)
     */
    public void add(Servo servo, double targetPos, long durationMs, boolean interpolate) {
        actionQueue.add(new ServoAction(servo, ServoUtil.clamp(targetPos), durationMs, interpolate));
    }

    /**
     * <p>Degrees used instead of servo position</p>
     * A Maximum Degree and Minimum Degree is required for accurate positions
     * @param servo Servo you want to move
     * @param targetPos_deg The target position in degrees
     * @param durationMs Time to wait after finishing the action
     * @param maxDeg Maximum Degrees the servo can handle
     * @param minDeg Minimum Degrees the servo can handle
     * @see #add(Servo, double, long)
     * @see #add(Servo, double, long, boolean)
     * @see #add(Servo, int, long, int, int, boolean)
     */
    public void add(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg) {
        double targetPos = ServoUtil.degToPos(targetPos_deg, maxDeg, minDeg);
        add(servo, ServoUtil.clamp(targetPos), durationMs);
    }

    /**
     * <p>Degrees used instead of servo position with optional interpolation</p>
     * A Maximum Degree and Minimum Degree is required for accurate positions
     * @param servo Servo you want to move
     * @param targetPos_deg The target position in degrees
     * @param durationMs Time to interpolate over
     * @param maxDeg Maximum Degrees the servo can handle
     * @param minDeg Minimum Degrees the servo can handle
     * @param interpolate Whether to interpolate or not
     * @see #add(Servo, double, long)
     * @see #add(Servo, double, long, boolean)
     * @see #add(Servo, int, long, int, int)
     */
    public void add(Servo servo, int targetPos_deg, long durationMs, int maxDeg, int minDeg, boolean interpolate) {
        double targetPos = ServoUtil.degToPos(targetPos_deg, maxDeg, minDeg);
        add(servo, ServoUtil.clamp(targetPos), durationMs, interpolate);
    }

    /**
     * Adds a Motor Action to Queue
     * <p>Moves a single motor</p>
     * @param motor Motor you want to move
     * @param power Direction and speed to run [-1.0 - 1.0]
     * @param durationMs How long to run
     */
    public void add(DcMotor motor, double power, long durationMs) {
        actionQueue.add(new MotorAction(motor, power, durationMs));
    }

    public void add(DcMotor m1, double p1, DcMotor m2, double p2, long durationMs) {
        actionQueue.add(new DualMotorAction(m1, p1, m2, p2, durationMs));
    }


    /**
     * Adds a Delay Action to Queue
     * <p>A Non-blocking wait within the Sequencer</p>
     * @param durationMs How long to wait
     */
    public void add(long durationMs) {
        actionQueue.add(new Delay(durationMs));
    }

    /**
     * General fallback if action has no overload
     * @param action A SeqAction class
     * @see #add(Servo, double, long)
     * @see #add(Servo, double, long, boolean)
     * @see #add(Servo, int, long, int, int)
     * @see #add(Servo, int, long, int, int, boolean)
     * @see #add(DcMotor, double, long)
     * @see #add(long)
     */
    public void add(SeqAction action) {
        actionQueue.add(action);
    }

    /**
     * Call repeatedly inside loop()
     */
    public void step() {
        if (currentAction == null) { // not executing an action
            currentAction = actionQueue.poll();
            if (currentAction == null) return; // Queue is empty
            if (currentAction instanceof ServoAction && ((ServoAction) currentAction).interpolate) {
                ((ServoAction) currentAction).startPos = ((ServoAction) currentAction).servo.getPosition();
            }

            timer.reset();
        }

        if (currentAction.execute()) {
            currentAction = null;
            actionCounter++;
        }
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

    /**
     * Single motor movement
     */
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

    public final class DualMotorAction implements SeqAction {
        public final DcMotor m1, m2;
        public final double p1, p2;
        public final long durationMs;

        public DualMotorAction(DcMotor m1, double p1, DcMotor m2, double p2, long durationMs) {
            this.m1 = m1;
            this.p1 = p1;
            this.m2 = m2;
            this.p2 = p2;
            this.durationMs = durationMs;
        }

        @Override
        public boolean execute() {
            m1.setPower(Math.min(1, Math.max(-1, p1)));
            m2.setPower(Math.min(1, Math.max(-1, p2)));

            if (timer.milliseconds() >= durationMs) {
                m1.setPower(0);
                m2.setPower(0);
                return true;
            }
            return false;
        }
    }


    /**
     * Waits the specified milliseconds
     */
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