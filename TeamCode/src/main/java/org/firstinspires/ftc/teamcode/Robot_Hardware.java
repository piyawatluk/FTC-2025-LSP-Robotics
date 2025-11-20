package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public String rightShooterMotorName;
    public String leftShooterMotorName;
    public String leftBeltDriveMotorName;
    public String rightBeltDriveMotorName;
    public String liftMotorName;

    public DcMotor frontLeftMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor rightShooterMotor = null;
    public DcMotor leftShooterMotor = null;
    public DcMotor leftBeltDriveMotor = null;
    public DcMotor rightBeltDriveMotor = null;
    public DcMotor liftMotor = null;

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

        rightShooterMotorName = prop.getProperty("Robot.RIGHT_SHOOTER_MOTOR_NAME", "rsm");
        leftShooterMotorName = prop.getProperty("Robot.LEFT_SHOOTER_MOTOR_NAME", "lsm");

        rightBeltDriveMotorName = prop.getProperty("Robot.RIGHT_BELT_DRIVE_MOTOR_NAME","rbdm");
        //leftBeltDriveMotorName = prop.getProperty("Robot.LEFT_BELT_DRIVE_MOTOR_NAME","lbdm");

        liftMotorName = prop.getProperty("Robot.LIFT_MOTOR_NAME","lft");



        placeholderServoName1 = prop.getProperty("Robot.PLACEHOLDER_SERVO1_NAME", "null");
        placeholderServoName2 = prop.getProperty("Robot.PLACEHOLDER_SERVO2_NAME", "null");
        placeholderServoName3 = prop.getProperty("Robot.PLACEHOLDER_SERVO3_NAME", "null");
        placeholderServoName4 = prop.getProperty("Robot.PLACEHOLDER_SERVO4_NAME", "null");

        // map hardware
        frontLeftMotor = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        rearLeftMotor = hardwareMap.get(DcMotor.class, rearLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotor.class, frontRightMotorName);
        rearRightMotor = hardwareMap.get(DcMotor.class, rearRightMotorName);

        rightShooterMotor = hardwareMap.get(DcMotor.class, rightShooterMotorName);
        leftShooterMotor = hardwareMap.get(DcMotor.class, leftShooterMotorName);

        rightBeltDriveMotor = hardwareMap.get(DcMotor.class, rightBeltDriveMotorName);
        //leftBeltDriveMotor = hardwareMap.get(DcMotor.class, leftBeltDriveMotorName);

        liftMotor = hardwareMap.get(DcMotor.class, liftMotorName);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightBeltDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBeltDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD); //tbd
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); //tbd
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBeltDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);//tbd
        //leftBeltDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //tbd


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
