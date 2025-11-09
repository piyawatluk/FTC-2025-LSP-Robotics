package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.AprilTagEasyHelper; // <-- Add this import

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private Servo servo1;
    private Servo servo2;
    Sequencer Sequence1 = new Sequencer();
    Sequencer Sequence2 = new Sequencer();

    // Add helper for AprilTag detection
    private AprilTagEasyHelper aprilTagHelper;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Robot_Hardware robotHardware = new Robot_Hardware();
        robotHardware.init(hardwareMap,telemetry);

        mecanumDrive = new MecanumDrive(robotHardware);
        servo1 = robotHardware.placeholderServo1;
        servo2 = robotHardware.placeholderServo2;

        // Initialize AprilTag helper (true = use webcam, name = "Webcam 1")
        aprilTagHelper = new AprilTagEasyHelper(true, "Webcam 1");
        aprilTagHelper.initialize(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        Sequence1.add(servo1, 0.5, 2000);
        Sequence1.add(servo1, 0.2, 600, true);
        Sequence2.add(servo2, 0.6, 100);
        Sequence2.add(830);

        runtime.reset();
    }

    @Override
    public void loop() {
        mecanumDrive.drive(gamepad1);
        Sequence1.step();
        if (Sequence1.actionCounter > 1) {
            Sequence2.step();
        }

        // Use AprilTag detections (example: show on telemetry)
        if (aprilTagHelper != null) {
            List<AprilTagDetection> detections = aprilTagHelper.getDetections();
            telemetry.addData("# AprilTags Detected", detections.size());
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                }
            }
        }
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            aprilTagHelper.shutdown();
        }
    }
}