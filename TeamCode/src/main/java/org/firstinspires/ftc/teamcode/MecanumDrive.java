package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive {

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private final Robot_Hardware hardware;

    public MecanumDrive(Robot_Hardware hardware) {
        this.hardware = hardware;
    }

    public void drive(Gamepad gamepad) {
        double computedX_l = CoordinateConverter.computeX(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedY_l = CoordinateConverter.computeY(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedX_r = CoordinateConverter.computeX(gamepad.right_stick_x, gamepad.right_stick_y, 1);

        // Calculate motor powers
        frontLeftPower  = (computedY_l - computedX_l) + computedX_r;
        frontRightPower = (computedY_l + computedX_l) - computedX_r;
        backLeftPower   = (computedY_l + computedX_l) + computedX_r;
        backRightPower  = (computedY_l - computedX_l) - computedX_r;

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
}
