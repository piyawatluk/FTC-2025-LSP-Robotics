package org.firstinspires.ftc.teamcode.util;

public class AreaLimiter {
    // Allowed virtual box (inches)
    public double X_MIN = -60;
    public double X_MAX =  60;
    public double Y_MIN =   0;
    public double Y_MAX =  72;

    public double[] limit(double x, double y, double driveX, double driveY) {

        // Left wall
        if (x <= X_MIN && driveX < 0) driveX = 0;
        // Right wall
        if (x >= X_MAX && driveX > 0) driveX = 0;

        // Bottom wall
        if (y <= Y_MIN && driveY < 0) driveY = 0;
        // Top wall
        if (y >= Y_MAX && driveY > 0) driveY = 0;

        return new double[]{driveX, driveY};
    }
}
