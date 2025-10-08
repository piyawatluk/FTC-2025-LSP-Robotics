package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive {
    private final Robot_Hardware hardware;

    public MecanumDrive(Robot_Hardware hardware) {
        this.hardware = hardware;
    }

    public void drive(Gamepad gamepad) {
        double computedX_l = CoordinateConverter.computeX(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedY_l = CoordinateConverter.computeY(gamepad.left_stick_x, gamepad.left_stick_y, 1);
        double computedX_r = CoordinateConverter.computeX(gamepad.right_stick_x, gamepad.right_stick_y, 1);

        double frontLeft = (computedY_l - computedX_l) + computedX_r;
        double frontRight = (computedY_l + computedX_l) - computedX_r;
        double backLeft = (computedY_l + computedX_l) + computedX_r;
        double backRight = (computedY_l - computedX_l) - computedX_r;

        hardware.frontLeftMotor.setPower(frontLeft);
        hardware.frontRightMotor.setPower(frontRight);
        hardware.rearLeftMotor.setPower(backLeft);
        hardware.rearRightMotor.setPower(backRight);
    }
}
