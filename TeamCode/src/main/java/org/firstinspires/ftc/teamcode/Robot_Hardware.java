package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public Robot_Hardware() { }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Load config on object creation
        Properties prop = new Properties();
        try(InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")){
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null) {
                telemetry.addData("ERROR", "Cannot read assets/Robot.config");
            }
            return;
        }

        frontLeftMotorName = prop.getProperty("Robot.FRONT_LEFT_MOTOR_NAME", "flm");
        frontRightMotorName = prop.getProperty("Robot.FRONT_RIGHT_MOTOR_NAME", "frm");
        rearLeftMotorName = prop.getProperty("Robot.REAR_LEFT_MOTOR_NAME", "rlm");
        rearRightMotorName = prop.getProperty("Robot.REAR_RIGHT_MOTOR_NAME", "rrm");


        /*
        placeholderServoName1 = prop.getProperty("Robot.PLACEHOLDER_SERVO1_NAME", "null");
        placeholderServoName2 = prop.getProperty("Robot.PLACEHOLDER_SERVO2_NAME", "null");
        placeholderServoName3 = prop.getProperty("Robot.PLACEHOLDER_SERVO3_NAME", "null");
        placeholderServoName4 = prop.getProperty("Robot.PLACEHOLDER_SERVO4_NAME", "null");
        */

        // map hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        rearLeftMotor = hardwareMap.get(DcMotor.class, rearLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotor.class, frontRightMotorName);
        rearRightMotor = hardwareMap.get(DcMotor.class, rearRightMotorName);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        /*
        placeholderServo1 = hardwareMap.get(Servo.class, placeholderServoName1);
        placeholderServo2 = hardwareMap.get(Servo.class, placeholderServoName2);
        placeholderServo3 = hardwareMap.get(Servo.class, placeholderServoName3);
        placeholderServo4 = hardwareMap.get(Servo.class, placeholderServoName4);

        placeholderServo1.setDirection(Servo.Direction.FORWARD);
        placeholderServo2.setDirection(Servo.Direction.FORWARD);
        placeholderServo3.setDirection(Servo.Direction.FORWARD);
        placeholderServo4.setDirection(Servo.Direction.FORWARD);
        */
    }
}
