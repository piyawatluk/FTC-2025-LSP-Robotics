package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive_own mecanumDriveOwn;
    private MecanumDrive md;
    private boolean prevA = false;
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);
    private AreaLimiter areaLimiter;
    private AprilTagEasyHelper aprilTagHelper;

    @Override
    public void init() {
        areaLimiter = new AreaLimiter(telemetry);
        telemetry.addData("Status", "Initialized");
        hw.init(hardwareMap, telemetry);
        md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        mecanumDriveOwn = new MecanumDrive_own(md);
        aprilTagHelper = new AprilTagEasyHelper(true, "Webcam 1");
        aprilTagHelper.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        final double INFF = 999999; //kinda like INF
        double distanceToAprilTag = INFF;
        boolean canShoot = false;

        double bearing = 0;


        List<AprilTagDetection> detections = aprilTagHelper.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            if (detection != null && detection.ftcPose != null) {
                distanceToAprilTag = Math.sqrt(
                        detection.ftcPose.x * detection.ftcPose.x +
                                detection.ftcPose.y * detection.ftcPose.y
                );
                //telemetry.addLine(String.format("ID %d: %s", detection.id, detection.metadata.name));
                //telemetry.addLine(String.format(" XYZ (in)  %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format(" PRY (deg) %6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addData("test baring", detection.ftcPose.bearing);
                bearing = detection.ftcPose.bearing;
                if (distanceToAprilTag >= 67) canShoot = true;
            } else {
                telemetry.addLine("AprilTag detected but pose unavailable.");
            }
        } else {
            telemetry.addLine("April Tag not detected");
        }

        md.updatePoseEstimate();
        Pose2d pose = md.localizer.getPose();
        double factor = 0;
        double x = pose.position.x;
        double y = pose.position.y;
        double heading = pose.heading.toDouble();
        boolean inTriangle = areaLimiter.inShootingZone(x, y);

        double rawLX = gamepad1.left_stick_x; // strafe (left/right)
        double rawLY = -gamepad1.left_stick_y; // forward/back (invert so up = +)
        double turn = gamepad1.right_stick_x + factor; // rotation

        double[] limited = areaLimiter.limit(x, y, rawLY, rawLX);
        double limitedLX = limited[0];
        double limitedLY = limited[1];

        areaLimiter.hardWall(!gamepad1.left_bumper && !gamepad1.right_bumper);

        /* remove the comments and put the comments on driveLimited if you want to run a normal teleop(NotLimited)
        right now the mecanum drive got 2 drive system na you can actually delete the other one 'drive()' but it can stay there just for testing and stuff*/

        mecanumDriveOwn.driveLimited(limitedLX, limitedLY, turn);
        //mecanumDriveOwn.drive(gamepad1);

        //util.servo_test(hardwareMap, startSequence, telemetry);
        if (aprilTagHelper != null){
            util.shooter(gamepad1.b, (distanceToAprilTag / 150) * 6000); //dummy value: subject to change
        }
        else util.shooter(gamepad1.b, 3000); //dummy value: subject to change

        util.feeder(gamepad1.b);
        util.lift(gamepad1.x, telemetry);


        telemetry.addLine("LSP Robotic Senior - Teleop");
        telemetry.addData("Left front motor speed", mecanumDriveOwn.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDriveOwn.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDriveOwn.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDriveOwn.getMotorPower("RBM"));
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        if (aprilTagHelper != null){
            if (distanceToAprilTag < INFF) {
                telemetry.addData("Distance to April Tag", distanceToAprilTag);
            } else {
                telemetry.addLine("April Tag not detected");
            }
            telemetry.addData("In shooting range", canShoot);
            telemetry.addData("In Shooting Area (Front)", inTriangle);
            telemetry.addData("returnval", (util.aimmer(gamepad1.a,bearing,heading,telemetry)));
            if (canShoot && inTriangle) {
                telemetry.addLine("GO SHOOT!!!");
            } else {
                telemetry.addLine("DO NOT SHOOT!!!");
            }
        }
        else telemetry.addLine("camera assist not available");

        telemetry.addData("limited X", limitedLX);
        telemetry.addData("limited Y", limitedLY);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            aprilTagHelper.shutdown();
        }
    }

}