package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.generalUtil;
import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.AprilTagEasyHelper;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private boolean prevA = false;
    private boolean prevB = false;
    private Sequencer belt = new Sequencer();
    public static DcMotor leftBeltDriveMotor;
    public static DcMotor rightBeltDriveMotor;
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);


    // AprilTag helper
    private AprilTagEasyHelper aprilTagHelper;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        hw.init(hardwareMap, telemetry);

        mecanumDrive = new MecanumDrive(hw);

        // Initialize AprilTag helper: change useWebcam/name if you want phone camera instead
        aprilTagHelper = new AprilTagEasyHelper(true, "Webcam 1");
        aprilTagHelper.initialize(hardwareMap);
    }

    @Override
    public void start() {

        runtime.reset();
    }

    @Override
    public void loop() {
        mecanumDrive.drive(gamepad1);

        boolean currentA = gamepad1.a;
        //boolean currentB = gamepad1.b;

        // Start the sequence once per press
        boolean startSequence = currentA && !prevA;
        //boolean lift_logic = currentB && !prevB;

        util.servo_test(hardwareMap, startSequence, telemetry);
        util.belt(gamepad1.b);

        prevA = currentA;
        //prevB = currentB;



        // Allow toggling camera streaming to save CPU if needed
        if (gamepad1.dpad_down) {
            if (aprilTagHelper != null) aprilTagHelper.stopStreaming();
        } else if (gamepad1.dpad_up) {
            if (aprilTagHelper != null) aprilTagHelper.resumeStreaming();
        }

        // Display AprilTag detections on telemetry
        if (aprilTagHelper != null) {
            List<AprilTagDetection> detections = aprilTagHelper.getDetections();
            telemetry.addData("# AprilTags Detected", detections.size());
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.ftcPose != null) {
                    telemetry.addLine(String.format("ID %d: %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format(" XYZ (in)  %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(" PRY (deg) %6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                } else {
                    telemetry.addLine(String.format("ID %d: Unknown  Center (px) %6.0f %6.0f", detection.id, detection.center.x, detection.center.y));
                    telemetry.addData("Metadata", detection.metadata);
                    telemetry.addData("ftcPose" , detection.ftcPose);
                }
            }
        }

        telemetry.addLine("LSP Robotic Senior - Teleop");
        telemetry.addData("Left front motor speed", mecanumDrive.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDrive.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDrive.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDrive.getMotorPower("RBM"));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            aprilTagHelper.shutdown();
        }
    }
}