package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AreaLimiter {
    private Telemetry telemetry;
    // Allowed virtual box (inches)

    public AreaLimiter(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public double X_MIN = -60;
    public double X_MAX =  60;
    public double Y_MIN =  -72;
    public double Y_MAX =  72;
    public boolean softWall = false;
    public boolean hardWall = true;
    public double DisplaceX;
    public double DisplaceY;
    public double Buff_range = 10; //Buffer

    public double[] limit(double x, double y, double driveX, double driveY) {
        //telemetry.addData("Current X", x);
        //telemetry.addData("Current Y", y); //for debug purposes
        // Left wall
        if (x <= X_MIN && driveX > 0 && hardWall) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }
        // Right wall
        if (x >= X_MAX && driveX < 0 && hardWall) {
            driveX = 0;
            telemetry.addData("Limit reached (X axis)", driveX);
        }

        // Bottom wall
        if (y <= Y_MIN && driveY < 0 && hardWall) {
            driveY = 0;
            telemetry.addData("Limit reached (Y axis)", driveY);
        }
        // Top wall
        if (y >= Y_MAX && driveY > 0 && hardWall) {
            driveY = 0;
            telemetry.addData("Softwall reached (Y axis)", driveY);
        }

        //Soft wall (Buffer zone)


        if (x <= X_MIN+Buff_range && driveX > 0 && softWall) {
            while (x >= X_MIN){
                DisplaceX = x-X_MIN;
                driveX = DisplaceX/10;
            }
            telemetry.addData("Softwall reached (X axis)", driveX);
        }
        // Right wall
        if (x >= X_MAX-Buff_range && driveX < 0 && softWall) {
            while (x <= X_MAX){
                DisplaceX = X_MAX-x;
                driveX = DisplaceX/10;
            }
            telemetry.addData("Softwall reached (X axis)", driveX);
        }

        // Bottom wall
        if (y <= Y_MIN+Buff_range && driveY < 0 && softWall) {
            while (y >= Y_MIN){
                DisplaceY = y-Y_MIN;
                driveY = DisplaceY/10;
            }
            telemetry.addData("Softwall reached (Y axis)", driveY);
        }
        // Top wall
        if (y >= Y_MAX-Buff_range && driveY > 0 && softWall) {
            while (y <= Y_MAX){
                DisplaceY = Y_MAX-y;
                driveY = DisplaceY/10;
            }
            telemetry.addData("Softwall reached (Y axis)", driveY);
        }

        return new double[]{driveX, driveY};
    }

    public void hardWall(boolean b) {
        hardWall = true;
    }
    public Boolean inShootingZone (double x, double y)
    {
        //note this assumes that the center is (0,0) and the obelisk is at x-positive just tell me if i'm wrong
        // this also only applies to the top shooting zone since the bottom seems unlikely
        return y <= x && y >= -x && x >= 0;
    }
}
