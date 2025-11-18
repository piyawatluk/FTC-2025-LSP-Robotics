package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class AreaLimiter {
    // Allowed virtual box (inches)
    public double X_MIN = -60;
    public double X_MAX =  60;
    public double Y_MIN =   0;
    public double Y_MAX =  72;

    public double[] limit(double x, double y, double driveX, double driveY) {
        //telemetry.addData("Current X", x);
        //telemetry.addData("Current Y", y); //for debug purposes
        // Left wall
        if (x <= X_MIN && driveX < 0) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }
        // Right wall
        if (x >= X_MAX && driveX > 0) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }

        // Bottom wall
        if (y <= Y_MIN && driveY < 0) {
            driveY = 0;
            telemetry.addData("Limit reached (Y axis)", driveX);
        }
        // Top wall
        if (y >= Y_MAX && driveY > 0) {
            driveY = 0;
            telemetry.addData("Limit reached (Y axis)", driveX);
        }

        return new double[]{driveX, driveY};
    }
}
