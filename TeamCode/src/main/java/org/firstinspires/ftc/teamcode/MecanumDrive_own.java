package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive_own {

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private final double straff_dampening_factor = 0.7;
    private double yaw_dampening_factor = 0.5;

    private final MecanumDrive hardware;

    public MecanumDrive_own(MecanumDrive hardware) {
        this.hardware = hardware;
    }

    public void drive(Gamepad gamepad) {
        double computedX_l = CoordinateConverter.computeX(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedY_l = CoordinateConverter.computeY(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedX_r = CoordinateConverter.computeX(gamepad.right_stick_x, gamepad.right_stick_y, 1);

        // Calculate motor powers
        frontLeftPower  = (-computedY_l + computedX_l) + (0.5*computedX_r);
        frontRightPower = (computedY_l + computedX_l) + (0.5*computedX_r);
        backLeftPower   = (-computedY_l - computedX_l) + (0.5*computedX_r);
        backRightPower  = (computedY_l - computedX_l) + (0.5*computedX_r);

        // Normalize so values stay in [-1, 1]
        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        frontLeftPower  /= max;
        frontRightPower /= max;
        backLeftPower   /= max;
        backRightPower  /= max;

        // Apply to hardware
        hardware.leftFront.setPower(frontLeftPower);
        hardware.rightFront.setPower(frontRightPower);
        hardware.leftBack.setPower(backLeftPower);
        hardware.rightBack.setPower(backRightPower);
    }

    public double getMotorPower(String atr) {
        switch (atr) {
            case "LFM": return frontLeftPower;
            case "RFM": return frontRightPower;
            case "LBM": return backLeftPower;
            case "RBM": return backRightPower;
            default: return 0.0;
        }
    }

    public void driveLimited(double driveX, double driveY, double turn) {
        // Standard mecanum math
        float stickLX = (float) driveY;
        float stickLY = (float) -driveX;

        double computedX_l = CoordinateConverter.computeX(stickLX, stickLY, 1);
        double computedY_l = CoordinateConverter.computeY(stickLX, stickLY, 1);

        float stickRX = (float) turn;
        double computedX_r = CoordinateConverter.computeX(stickRX, 0f, 1);

        // Same motor mixing as in drive()
        double fl = (-computedY_l + computedX_l*straff_dampening_factor) + (yaw_dampening_factor*computedX_r);
        double fr = (computedY_l + computedX_l*straff_dampening_factor) + (yaw_dampening_factor*computedX_r);
        double bl = (-computedY_l - computedX_l*straff_dampening_factor) + (yaw_dampening_factor*computedX_r);
        double br = (computedY_l - computedX_l*straff_dampening_factor) + (yaw_dampening_factor*computedX_r);

        // Normalize so no wheel |power| > 1
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Save for telemetry
        frontLeftPower  = fl;
        frontRightPower = fr;
        backLeftPower   = bl;
        backRightPower  = br;

        // Send to motors
        hardware.leftFront.setPower(fl);
        hardware.rightFront.setPower(fr);
        hardware.leftBack.setPower(bl);
        hardware.rightBack.setPower(br);

    }
}
