package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Robot_Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.InputStream;
import java.util.Properties;

public class generalUtil {

    static Sequencer Sequence1 = new Sequencer();
    public static Servo servo1 = null;

    Robot_Hardware robotHardware = new Robot_Hardware();

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

    public static void servo_test(HardwareMap hardwareMap, boolean logic, Telemetry telemetry) {
        Servo servo1 = hardwareMap.get(Servo.class, "tbd_0");
        if (servo1 == null) {
            telemetry.addData("ERROR", "The servo is NULL");
        } else {
            Sequence1.add(servo1, 0.5, 2000);
            Sequence1.add(servo1, 0.2, 600, true);
            Sequence1.add(830);
            if (logic) {
                Sequence1.step();
            }
        }
    }
}
