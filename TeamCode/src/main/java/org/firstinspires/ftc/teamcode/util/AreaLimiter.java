package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AreaLimiter {
    private Telemetry telemetry;
    // Allowed virtual box (inches)

    public AreaLimiter(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public double X_MIN = -10;
    public double X_MAX =  10;
    public double Y_MIN =  -10;
    public double Y_MAX =  10;
    public boolean softWall = false;
    public boolean hardWall = false;
    public boolean WantToShoot = false;
    public double DisplaceX;
    public double DisplaceY;

    public double[] limit(double x, double y, double driveX, double driveY) {

        // Left wall – only block movement INTO the wall
        if (x <= X_MIN && driveX > 0 && hardWall) {
            driveX = 0;
            telemetry.addData("Bottom wall limit", x);
        }

        // Right wall
        if (x >= X_MAX && driveX < 0 && hardWall) {
            driveX = 0;
            telemetry.addData("Top wall limit", x);
        }

        // Bottom wall
        if (y <= Y_MIN && driveY < 0 && hardWall) {
            driveY = 0;
            telemetry.addData("Left wall limit", y);
        }

        // Top wall
        if (y >= Y_MAX && driveY > 0 && hardWall) {
            driveY = 0;
            telemetry.addData("Right wall limit", y);
        }

        // Shooting zone lock
        if (WantToShoot && (inShootingZone(x, y) || inFarShootZone(x, y))) {
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
