package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.InputStream;
import java.util.Properties;

public class generalUtil {

    static Sequencer Sequence1 = new Sequencer();
    static Sequencer lift_seq = new Sequencer();
    private final Robot_Hardware hardware;
    public static Servo servo1 = null;


    Robot_Hardware robotHardware = new Robot_Hardware();

    public generalUtil(Robot_Hardware hardware) {
        this.hardware = hardware;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Load config on object creation
        Properties prop = new Properties();
        try (InputStream input = hardwareMap.appContext.getAssets().open("Robot.config")) {
            prop.load(input);
        } catch (Exception e) {
            RobotLog.ee("Robot_Hardware", e, "Failed to load Robot.config");
            if (telemetry != null) {
                telemetry.addData("ERROR", "Cannot read assets/Robot.config");
            }
        }
    }

    public static void servo_test(HardwareMap hardwareMap, boolean start, Telemetry telemetry) {
        Servo servo1 = hardwareMap.get(Servo.class, "tbd_0");

        if (servo1 == null) {
            telemetry.addData("ERROR", "The servo is NULL");
            return;
        }

        // When the button is pressed (start == true), define the sequence
        if (start) {
            Sequence1 = new Sequencer();
            Sequence1.add(servo1, 0.5, 500);
            Sequence1.add(servo1, 0.2, 500, true);
            Sequence1.add(830);
        }

        // Always step the sequence each loop (it runs automatically)
        Sequence1.step();
    }

    public void lift(HardwareMap hardwareMap, boolean start, Telemetry telemetry){

        final DcMotor liftMotor = hardware.liftMotor;
        if (start){
            lift_seq =new Sequencer();
            lift_seq.
            liftMotor.setTargetPosition(1900);
        }


    }
}
