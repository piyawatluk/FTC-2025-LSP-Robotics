package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtil {
     public void moveServoIncrement(Servo servo, double n) {
        safeSetPosition(servo, servo.getPosition() + n);
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
         servo.setPosition(clamp(pos));
    }

    public static void center(Servo servo) {
         servo.setPosition(0.5);
    }
}
