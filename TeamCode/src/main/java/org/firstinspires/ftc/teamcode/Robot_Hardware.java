package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    // Rename the servos
    public String placeholderServoName1;
    public String placeholderServoName2;
    public String placeholderServoName3;
    public String placeholderServoName4;

    public Servo placeholderServo1 = null;
    public Servo placeholderServo2 = null;
    public Servo placeholderServo3 = null;
    public Servo placeholderServo4 = null;

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

        frontLeftMotorName = prop.getProperty("Robot.FRONT_LEFT_MOTOR_NAME", "null");
        frontRightMotorName = prop.getProperty("Robot.FRONT_RIGHT_MOTOR_NAME", "null");
        rearLeftMotorName = prop.getProperty("Robot.REAR_LEFT_MOTOR_NAME", "null");
        rearRightMotorName = prop.getProperty("Robot.REAR_RIGHT_MOTOR_NAME", "null");


        placeholderServoName1 = prop.getProperty("Robot.PLACEHOLDER_SERVO1_NAME", "null");
        placeholderServoName2 = prop.getProperty("Robot.PLACEHOLDER_SERVO2_NAME", "null");
        placeholderServoName3 = prop.getProperty("Robot.PLACEHOLDER_SERVO3_NAME", "null");
        placeholderServoName4 = prop.getProperty("Robot.PLACEHOLDER_SERVO4_NAME", "null");

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


        placeholderServo1 = hardwareMap.get(Servo.class, placeholderServoName1);
        placeholderServo2 = hardwareMap.get(Servo.class, placeholderServoName2);
        placeholderServo3 = hardwareMap.get(Servo.class, placeholderServoName3);
        placeholderServo4 = hardwareMap.get(Servo.class, placeholderServoName4);

        placeholderServo1.setDirection(Servo.Direction.FORWARD);
        placeholderServo2.setDirection(Servo.Direction.FORWARD);
        placeholderServo3.setDirection(Servo.Direction.FORWARD);
        placeholderServo4.setDirection(Servo.Direction.FORWARD);

    }
}
