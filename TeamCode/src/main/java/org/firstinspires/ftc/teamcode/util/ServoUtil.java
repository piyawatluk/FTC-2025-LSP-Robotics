package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtil {
    public void moveIncrement(Servo servo, double pos) {
        safeSetPosition(servo, servo.getPosition() + pos);
    }

    public void moveIncrement(Servo servo, int deg, int maxDeg, int minDeg) {
        moveIncrement(servo, degToPos(deg, maxDeg, minDeg));
    }

    public static double degToPos(int deg, int maxDeg, int minDeg) {
        return (double) (deg - minDeg) / (maxDeg - minDeg);
    }

    public static int posToDeg(double pos, int maxDeg, int minDeg) {
        return (int) (pos * (maxDeg - minDeg) + minDeg);
    }

    public static double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    public void safeSetPosition(Servo servo, double pos) {
        servo.setPosition(clamp(pos)); // won't explode when pos is <1.0 and >0.0
    }

    public static void servoPresets(Servo servo, int n) {
        n = Math.min(5, Math.max(1, n));
        switch (n) { // assume 0 min and 180 max deg
            case 1: servo.setPosition(0.0); break;    // 0 deg
            case 2: servo.setPosition(0.25); break; // 45 deg
            case 3: servo.setPosition(0.5); break;  // 90 deg
            case 4: servo.setPosition(0.75); break; // 135 deg
            case 5: servo.setPosition(1.0); break;    // 180 deg
        }
    }
}
