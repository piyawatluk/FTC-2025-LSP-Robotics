package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Road-Runner follower Ziegler–Nichols tuner.
 *  - run this opmode and increase Kp until steady oscillation.
 *  - When steady oscillation occurs: kP_test = Ku, displayed Pu = oscillation period.
 *  - Put the computed Z-N PID value into Translation PID in SampleMacanumDrive.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForthZiegler extends LinearOpMode {

    // Slide this in Dashboard to tune. Start at 0 and increase slowly.
    public static double kP_test = 0.0;

    // Imaginary target 24 inch away
    public static double TARGET_X = 24.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Prepare: ensure TRANSLATIONAL_PID I and D = 0 (or set to 0 here).");
        telemetry.addLine("Use Dashboard to increase kP_test until steady oscillation.");
        telemetry.update();

        // Build a simple forward trajectory to the target (robot assumes start pose = 0).
        Trajectory trajToTarget = drive.trajectoryBuilder(new Pose2d())
                .forward(TARGET_X)
                .build();

        waitForStart();
        drive.followTrajectoryAsync(trajToTarget);

        double lastError = 0.0;
        double lastCrossingTime = -1.0;
        double Pu = 0.0;     // measured oscillation period
        double Ku = 0.0;     // ultimate gain

        while (opModeIsActive() && !isStopRequested()) {
            SampleMecanumDrive.TRANSLATIONAL_PID = new PIDCoefficients(kP_test, 0.0, 0.0);
            drive.update();

            double currentX = drive.getPoseEstimate().getX();
            double error = TARGET_X - currentX;
            double now = getRuntime();

            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                if (lastCrossingTime > 0) {
                    Pu = now - lastCrossingTime;
                    Ku = kP_test;
                }
                lastCrossingTime = now;
            }
            lastError = error;

            // 4) Send live telemetry to Dashboard + Driver Station
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("kP_test (Ku candidate)", kP_test);
            packet.put("Error (inches)", error);
            packet.put("Pu (sec)", Pu);
            packet.put("Ku (if steady)", Ku);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("kP_test (Ku?)", kP_test);
            telemetry.addData("Error (in)", error);
            telemetry.addData("Pu (s)", Pu);
            telemetry.addData("Ku (if steady)", Ku);

            //Compute and show Ziegler–Nichols PID
            if (Pu > 0 && Ku > 0) {
                double Kp = 0.6 * Ku;
                double Ki = 2.0 * Kp / Pu;
                double Kd = (Kp * Pu) / 8.0;
                telemetry.addData("Z-N Kp", "%.4f", Kp);
                telemetry.addData("Z-N Ki", "%.4f", Ki);
                telemetry.addData("Z-N Kd", "%.4f", Kd);
            }
            telemetry.update();
        }
    }
}
