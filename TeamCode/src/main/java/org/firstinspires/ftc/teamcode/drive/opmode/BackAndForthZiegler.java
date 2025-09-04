package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

// run this to figure out what is the ideal Ku and Pu then put in the Nichols-Ziegler urself in calculator or something
@Config
@Autonomous(group = "drive")
public class BackAndForthZiegler extends LinearOpMode {

    public static double kP_test = 0.0;
    public static double TARGET_X = 24.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Set kI = 0, kD = 0 in SampleMecanumDrive before ur gonna run");
        telemetry.update();

        waitForStart();

        double lastError = 0;
        double firstCrossingTime = -1;
        double Pu = 0;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP_test, 0, 0, 0)
                );

            double currentX = drive.getPoseEstimate().getX();
            double error = TARGET_X - currentX;
            double now = getRuntime();


            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                if (lastCrossingTime > 0) {
                    Pu = now - lastCrossingTime;
                }
                lastCrossingTime = now;
            }
            lastError = error;

            // Send data
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("kP_test", kP_test);
            packet.put("Error", error);
            packet.put("Pu (sec)", Pu);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Test kP", kP_test);
            telemetry.addData("Error", error);
            telemetry.addData("Pu (sec)", Pu);
            telemetry.update();
        }
    }
}
