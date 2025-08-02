package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.InputStream;
import java.util.Properties;

@TeleOp(name="Teleop", group="Iterative OpMode")
public class Main_Teleop extends OpMode
{
    private static final Properties prop = new Properties();
    public static String FRONT_LEFT_MOTOR_NAME = "null";
    public static String REAR_LEFT_MOTOR_NAME = "null";
    public static String FRONT_RIGHT_MOTOR_NAME = "null";
    public static String REAR_RIGHT_MOTOR_NAME = "null";



    static {
        try (InputStream input = CoordinateConverter.class.getResourceAsStream("/Robot.config")) {
            if (input != null) {
                prop.load(input);
            } else {
                System.err.println("Robot.config not found.");
            }
        } catch (Exception e) {
            System.err.println("Failed to load Robot.config: " + e.getMessage());
        }

        try {
            FRONT_LEFT_MOTOR_NAME = prop.getProperty("Robot.FRONT_LEFT_MOTOR_NAME", "null");
        } catch (NumberFormatException e) {
            System.err.println("Invalid saturation format. Using default.");
        }

        try {
            FRONT_RIGHT_MOTOR_NAME = prop.getProperty("Robot.FRONT_RIGHT_MOTOR_NAME", "null");
        } catch (NumberFormatException e) {
            System.err.println("Invalid saturation format. Using default.");
        }

        try {
            REAR_RIGHT_MOTOR_NAME = prop.getProperty("Robot.REAR_RIGHT_MOTOR_NAME", "null");
        } catch (NumberFormatException e) {
            System.err.println("Invalid saturation format. Using default.");
        }

        try {
            REAR_LEFT_MOTOR_NAME = prop.getProperty("Robot.REAR_LEFT_MOTOR_NAME", "null");
        } catch (NumberFormatException e) {
            System.err.println("Invalid saturation format. Using default.");
        }



    }
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRONT_LEFT_MOTOR = null;
    private DcMotor BACK_LEFT_MOTOR = null;
    private DcMotor FRONT_RIGHT_MOTOR = null;
    private DcMotor BACK_RIGHT_MOTOR = null;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        // Initialize motors
        FRONT_LEFT_MOTOR = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        BACK_LEFT_MOTOR = hardwareMap.get(DcMotor.class, REAR_LEFT_MOTOR_NAME);

        FRONT_RIGHT_MOTOR = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        BACK_RIGHT_MOTOR = hardwareMap.get(DcMotor.class, REAR_RIGHT_MOTOR_NAME);

        BACK_LEFT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BACK_RIGHT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRONT_LEFT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRONT_RIGHT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor directions
        FRONT_LEFT_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        BACK_LEFT_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        FRONT_RIGHT_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BACK_RIGHT_MOTOR.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {}
}