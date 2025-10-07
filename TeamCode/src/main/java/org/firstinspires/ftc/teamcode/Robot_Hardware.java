package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.InputStream;
import java.util.Properties;

public class Robot_Hardware {

    public String frontLeftMotorName;
    public String rearLeftMotorName;
    public String frontRightMotorName;
    public String rearRightMotorName;

    public DcMotor frontLeftMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearRightMotor = null;

    public Robot_Hardware() {
        // Load config on object creation
        Properties prop = new Properties();
        try (InputStream input = Robot_Hardware.class.getResourceAsStream("/Robot.config")) {
            if (input != null) {
                prop.load(input);
            } else {
                System.err.println("Robot.config not found.");
            }
        } catch (Exception e) {
            System.err.println("Failed to load Robot.config: " + e.getMessage());
        }

        frontLeftMotorName = prop.getProperty("Robot.FRONT_LEFT_MOTOR_NAME", "flm");
        frontRightMotorName = prop.getProperty("Robot.FRONT_RIGHT_MOTOR_NAME", "frm");
        rearLeftMotorName = prop.getProperty("Robot.REAR_LEFT_MOTOR_NAME", "rlm");
        rearRightMotorName = prop.getProperty("Robot.REAR_RIGHT_MOTOR_NAME", "rrm");
    }

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        rearLeftMotor = hardwareMap.get(DcMotor.class, rearLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotor.class, frontRightMotorName);
        rearRightMotor = hardwareMap.get(DcMotor.class, rearRightMotorName);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }
}
