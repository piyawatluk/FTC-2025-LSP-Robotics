package org.firstinspires.ftc.teamcode.util;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.InputStream;
import java.util.Properties;


public class generalUtil {

    static Sequencer Sequence1 = new Sequencer();
    private static Servo servo1;
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
            return;
        }

        servo1 = robotHardware.placeholderServo1;

    }
    public static void servo_test(boolean logic, Telemetry telemetry) {
        if (servo1 == null){
            telemetry.addData("servo1 is null nigger");
        }
        else{
            sequence1.add(servo1, 0.5, 2000);
            sequence1.add(servo1, 0.2, 600, true);
            sequence1.add(830);
            if (logic) {
                sequence1.step();
            }
        }

    }
}
