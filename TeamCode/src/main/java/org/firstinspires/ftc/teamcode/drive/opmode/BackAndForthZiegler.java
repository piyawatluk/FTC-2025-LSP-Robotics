package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// run this to figure out what is the ideal Ku and Pu then put in the Nichols-Ziegler urself in calculator or something
@Config
@Autonomous(group = "drive")
public class BackAndForthZiegler extends LinearOpMode {

    public static double kP_test = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Set kI = 0, kD = 0 in SampleMecanumDrive before ur gonna run");
        telemetry.update();

        waitForStart();

        double lastError = 0;
        double firstCrossingTime = -1;
        double period = 0;

        while (opModeIsActive() && !isStopRequested()) {
            double error = drive.getPoseEstimate().getX();
            double currentTime = getRuntime();

            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                if (firstCrossingTime < 0) {
                    firstCrossingTime = currentTime;
                } else {
                    period = currentTime - firstCrossingTime;
                    firstCrossingTime = currentTime;
                }
            }

            lastError = error;

            // Send data
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("kP_test", kP_test);
            packet.put("Error", error);
            packet.put("Oscillation Period", period);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Test kP", kP_test);
            telemetry.addData("Error", error);
            telemetry.addData("Oscillation Period", period);
            telemetry.update();
        }
    }
}
