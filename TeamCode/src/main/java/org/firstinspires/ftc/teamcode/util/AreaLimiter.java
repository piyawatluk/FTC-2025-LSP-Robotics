package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AreaLimiter {
    private final Telemetry telemetry;
    // Allowed virtual box (inches)

    public AreaLimiter(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public double X_MIN = -60;
    public double X_MAX =  60;
    public double Y_MIN =  -72;
    public double Y_MAX =  72;

    public boolean WantToShoot = false;

    public double[] limit(double x, double y, double driveX, double driveY) {
        //telemetry.addData("Current X", x);
        //telemetry.addData("Current Y", y); //for debug purposes
        // Left wall
        if (x <= X_MIN && driveX > 0) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }
        // Right wall
        if (x >= X_MAX && driveX < 0) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }

        // Bottom wall
        if (y <= Y_MIN && driveY < 0) {
            driveY = 0;
            telemetry.addData("Limit reached (Y axis)", driveY);
        }
        // Top wall
        if (y >= Y_MAX && driveY > 0) {
            driveY = 0;
            telemetry.addData("Limit reached (Y axis)", driveY);
        }
        //Shooting zone
        if (inShootingZone(x,y) && WantToShoot || inFarShootZone(x,y) && WantToShoot){
            driveX = 0;
            driveY = 0;
        }

        return new double[]{driveX, driveY};
    }
    public Boolean inShootingZone (double x, double y)
    {
        //note this assumes that the center is (0,0) and the obelisk is at x-positive just tell me if i'm wrong
        // this also only applies to the top shooting zone since the bottom seems unlikely
        return y <= x && y >= -x && x >= 0;
    }
    public boolean inFarShootZone(double x, double y) {
        double relX = x - 47;  // apex shift

        return relX >= 0 && relX <= 23 &&
                y >= -relX && y <= relX;
    }
}
