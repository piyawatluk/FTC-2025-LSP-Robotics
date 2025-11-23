package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

    private boolean prevY = false;
    private Sequencer belt = new Sequencer();
    public static DcMotor leftBeltDriveMotor;
    public static DcMotor rightBeltDriveMotor;
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);
    final double INFF = 999999; //kinda like INF
    double distanceToAprilTag = INFF;
    boolean canShoot = false;
    boolean shooter_Overwrite = false;
    boolean shootingAdapt = false;

    int adapt = 0;

    boolean slow = false;
    double slownum = 1;


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

        List<AprilTagDetection> detections = aprilTagHelper.getDetections();
        if(!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            distanceToAprilTag = sqrt(detection.ftcPose.x * detection.ftcPose.x + detection.ftcPose.y * detection.ftcPose.y);
            if (distanceToAprilTag >= 67) canShoot = true; //1.7m according to the most handsome guy whose name starts with W and ends in Y
        }

        double x = pose.position.x;
        double y = pose.position.y;
        if (gamepad1.dpad_down){
            slownum = 0.4;
        }

        if (gamepad1.dpad_up){
            slownum = 1;
        }


        double rawLX = gamepad1.left_stick_x;      // strafe (left/right)
        double rawLY = -gamepad1.left_stick_y;     // forward/back (invert so up = +)
        double turn  = (gamepad1.right_stick_x);     // rotation
        //turn *= slownum;


        if (gamepad1.y && !prevY) {
            areaLimiter.WantToShoot = !areaLimiter.WantToShoot;
        }
        prevY = gamepad1.y;

        double[] limited = areaLimiter.limit(x, y, rawLX, rawLY);
        double limitedLX = limited[0];
        double limitedLY = limited[1];

        areaLimiter.hardWall(!gamepad1.left_bumper && !gamepad1.right_bumper);



        /* remove the comments and put the comments on driveLimited if you want to run a normal teleop(NotLimited)
        right now the mecanum drive got 2 drive system na you can actually delete the other one 'drive()' but it can stay there just for testing and stuff*/

        //mecanumDriveOwn.driveLimited(limitedLX, limitedLY, turn);
        mecanumDriveOwn.drive(gamepad1,slownum);


        boolean currentA = gamepad1.a;
        //boolean currentB = gamepad1.b;
        // Start the sequence once per press
        boolean startSequence = currentA && !prevA;
        //boolean lift_logic = currentB && !prevB;

        //overwrite logic
        if (gamepad1.left_bumper){
            shooter_Overwrite = true;
            shootingAdapt = false;
        }
        if (gamepad1.right_bumper){
            shootingAdapt = true;
            shooter_Overwrite = false;
        }

        if (gamepad1.dpad_left){
            adapt += 10;

        }

        if (gamepad1.dpad_right){
            adapt -= 10;
        }

        //util.servo_test(hardwareMap, startSequence, telemetry);
        if (distanceToAprilTag < INFF){
            util.shooter(gamepad1.left_bumper,3000);
        }
        else if (shooter_Overwrite && !shootingAdapt || distanceToAprilTag > INFF) {
            util.shooter(gamepad1.left_bumper, 3000);
        }
        else if (shootingAdapt && !shooter_Overwrite){
            util.shooter(gamepad1.left_bumper, 3000);
        }

        util.feeder(gamepad1.a);
        util.lift(gamepad1.x, telemetry);
        util.the_gettho(gamepad1.left_trigger,gamepad1.right_trigger);



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




        boolean inTriangle = areaLimiter.inShootingZone(x,y);

        telemetry.addLine("LSP Robotic Senior - Teleop");
        telemetry.addData("Left front motor speed", mecanumDriveOwn.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDriveOwn.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDriveOwn.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDriveOwn.getMotorPower("RBM"));
        telemetry.addData("X",x);
        telemetry.addData("Y",y);
        if (distanceToAprilTag < INFF) {
            telemetry.addData("Distance to April Tag",distanceToAprilTag);
            telemetry.addData("target motor speed", (distanceToAprilTag/196)*6000);
        }

        else {telemetry.addLine("April Tag not detected");}
        telemetry.addData("In shooting range", canShoot);
        telemetry.addData("In Shooting Area (Front)", inTriangle);
        if (canShoot && inTriangle) telemetry.addLine("GO SHOOT!!!");
        else telemetry.addLine("DO NOT SHOOT!!!");

        telemetry.addData("Adaptive shooting speed : ",adapt);


        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            aprilTagHelper.shutdown();
        }
    }

}