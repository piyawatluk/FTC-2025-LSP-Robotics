package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive_own {

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private final Robot_Hardware hardware;

    public MecanumDrive_own(Robot_Hardware hardware) {
        this.hardware = hardware;
    }


    public void drive(Gamepad gamepad,double rspeed) {
        double computedX_l = CoordinateConverter.computeX(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedY_l = CoordinateConverter.computeY(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedX_r = CoordinateConverter.computeX(gamepad.right_stick_x, gamepad.right_stick_y, 1);

        // Calculate motor powers
        frontLeftPower  = (gamepad.left_stick_y - gamepad.left_stick_x) - gamepad.right_stick_x;
        frontRightPower = (gamepad.left_stick_y + gamepad.left_stick_x) + gamepad.right_stick_x;
        backLeftPower   = (gamepad.left_stick_y + gamepad.left_stick_x) - gamepad.right_stick_x;
        backRightPower  = (gamepad.left_stick_y - gamepad.left_stick_x) + gamepad.right_stick_x;

        // Normalize so values stay in [-1, 1]
        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        frontLeftPower  /= max;
        frontRightPower /= max;
        backLeftPower   /= max;
        backRightPower  /= max;

        frontLeftPower  *= rspeed;
        frontRightPower *= rspeed;
        backLeftPower   *= rspeed;
        backRightPower  *= rspeed;


        // Apply to hardware
        hardware.frontLeftMotor.setPower(frontLeftPower);
        hardware.frontRightMotor.setPower(frontRightPower);
        hardware.rearLeftMotor.setPower(backLeftPower);
        hardware.rearRightMotor.setPower(backRightPower);
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
        float stickLX = (float) driveX;
        float stickLY = (float) -driveY;

        double computedX_l = CoordinateConverter.computeX(stickLX, stickLY, 1);
        double computedY_l = CoordinateConverter.computeY(stickLX, stickLY, 1);

        float stickRX = (float) turn;
        double computedX_r = CoordinateConverter.computeX(stickRX, 0f, 1);

        // Same motor mixing as in drive()
        double fl = (computedY_l - computedX_l) - computedX_r;
        double fr = (computedY_l + computedX_l) + computedX_r;
        double bl = (computedY_l + computedX_l) - computedX_r;
        double br = (computedY_l - computedX_l) + computedX_r;

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
        hardware.frontLeftMotor.setPower(fl);
        hardware.frontRightMotor.setPower(fr);
        hardware.rearLeftMotor.setPower(bl);
        hardware.rearRightMotor.setPower(br);

    }
}