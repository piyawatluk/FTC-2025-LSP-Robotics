package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AreaLimiter;
import org.firstinspires.ftc.teamcode.util.generalUtil;
import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.AprilTagEasyHelper;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive_own mecanumDriveOwn;
    private MecanumDrive rrDrive;
    private Servo servo1;
    private Servo servo2;

    private Sequencer sequence1 = new Sequencer();
    private Sequencer sequence2 = new Sequencer();
    private boolean prevA = false;
    private boolean prevB = false;
    private Sequencer belt = new Sequencer();
    public static DcMotor leftBeltDriveMotor;
    public static DcMotor rightBeltDriveMotor;
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);


    //Area Limiter
    private AreaLimiter areaLimiter;

    // AprilTag helper
    private AprilTagEasyHelper aprilTagHelper;

    @Override
    public void init() {
        areaLimiter = new AreaLimiter(telemetry);

        telemetry.addData("Status", "Initialized");
        hw.init(hardwareMap, telemetry);

        mecanumDriveOwn = new MecanumDrive_own(hw);

        // Road Runner drive for pose
        rrDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        //rrDrive.setPoseEstimate(new Pose2d(0, 0, 0)); // starting pose

        // Initialize AprilTag helper: change useWebcam/name if you want phone camera instead
        aprilTagHelper = new AprilTagEasyHelper(true, "Webcam 1");
        aprilTagHelper.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        //Start counting Displacement For limiter
        //rrDrive.update();
        Pose2d pose = rrDrive.localizer.getPose();
        double x = pose.position.x;
        double y = pose.position.y;

        double rawLX = gamepad1.left_stick_x;      // strafe (left/right)
        double rawLY = -gamepad1.left_stick_y;     // forward/back (invert so up = +)
        double turn  = gamepad1.right_stick_x;     // rotation

        double[] limited = areaLimiter.limit(x, y, rawLX, rawLY);
        double limitedLX = limited[1];
        double limitedLY = limited[0];

        areaLimiter.hardWall(!gamepad1.left_bumper && !gamepad1.right_bumper);

        /* remove the comments and put the comments on driveLimited if you want to run a normal teleop(NotLimited)
        right now the mecanum drive got 2 drive system na you can actually delete the other one 'drive()' but it can stay there just for testing and stuff*/

        mecanumDriveOwn.driveLimited(limitedLX, limitedLY, turn);
        //mecanumDrive.drive(gamepad1);


        boolean currentA = gamepad1.a;
        //boolean currentB = gamepad1.b;

        // Start the sequence once per press
        boolean startSequence = currentA && !prevA;
        //boolean lift_logic = currentB && !prevB;

        util.servo_test(hardwareMap, startSequence, telemetry);
        util.shooter(gamepad1.b, 6000);
        util.lift(gamepad1.x, telemetry);

        prevA = currentA;
        //prevB = currentB;



        //// Allow toggling camera streaming to save CPU if needed
        //if (gamepad1.dpad_down) {
        //if (aprilTagHelper != null) aprilTagHelper.stopStreaming();
        //} else if (gamepad1.dpad_up) {
        //if (aprilTagHelper != null) aprilTagHelper.resumeStreaming();
        //}

        //// Display AprilTag detections on telemetry
        //if (aprilTagHelper != null) {
        //List<AprilTagDetection> detections = aprilTagHelper.getDetections();
        //telemetry.addData("# AprilTags Detected", detections.size());
        //for (AprilTagDetection detection : detections) {
        //if (detection.metadata != null && detection.ftcPose != null) {
        //telemetry.addLine(String.format("ID %d: %s", detection.id, detection.metadata.name));
        //telemetry.addLine(String.format(" XYZ (in)  %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        //telemetry.addLine(String.format(" PRY (deg) %6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        //} else {
        //telemetry.addLine(String.format("ID %d: Unknown  Center (px) %6.0f %6.0f", detection.id, detection.center.x, detection.center.y));
        //telemetry.addData("Metadata", detection.metadata);
        //telemetry.addData("ftcPose" , detection.ftcPose);
        //}
        //}
        //}

        telemetry.addLine("LSP Robotic Senior - Teleop");
        telemetry.addData("Left front motor speed", mecanumDriveOwn.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDriveOwn.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDriveOwn.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDriveOwn.getMotorPower("RBM"));
        telemetry.addData("X",x);
        telemetry.addData("Y",y);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            aprilTagHelper.shutdown();
        }
    }

}