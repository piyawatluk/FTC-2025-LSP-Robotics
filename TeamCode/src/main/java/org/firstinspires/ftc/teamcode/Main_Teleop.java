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
    boolean overwrite = false;
    boolean hardwall = true;
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);
    private AreaLimiter areaLimiter;
    private AprilTagEasyHelper aprilTagHelper;

    @Override
    public void init() {
        areaLimiter = new AreaLimiter(telemetry);
        telemetry.addData("Status", "Initialized");
        try {
            hw.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "hw.init threw: " + e.getMessage());
        }

        try {
            md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        } catch (Exception e) {
            telemetry.addData("Warning", "MecanumDrive init threw: " + e.getMessage());
            md = null;
        }

        try {
            if (md != null) {
                mecanumDriveOwn = new MecanumDrive_own(md);
            } else {
                mecanumDriveOwn = null;
            }
        } catch (Exception e) {
            telemetry.addData("Warning", "MecanumDrive_own init threw: " + e.getMessage());
            mecanumDriveOwn = null;
        }

        try {
            aprilTagHelper = new AprilTagEasyHelper(true, "Webcam 1");
            aprilTagHelper.initialize(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Warning", "AprilTagHelper init failed: " + e.getMessage());
            aprilTagHelper = null;
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("LSP Robotic Senior - Teleop");
        final double INFF = 999999; //kinda like INF
        double distanceToAprilTag = INFF;
        boolean canShoot = false;


        double bearing = 0; // initial value for bearing
        double manual_RPM = 3000; //TBD

        if (gamepad2.left_bumper && gamepad2.right_bumper){
            overwrite = true;
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper){
            hardwall = false;
        }

        // Safely get detections only if helper is available
        List<AprilTagDetection> detections = null;
        if (aprilTagHelper != null) {
            try {
                detections = aprilTagHelper.getDetections();
            } catch (Exception e) {
                telemetry.addData("AprilTagHelper.getDetections error", e.getMessage());
                detections = null;
            }
        } else {
            telemetry.addLine("aprilTagHelper is null; skipping vision");
        }

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            if (detection != null && detection.ftcPose != null) {
                try {
                    distanceToAprilTag = Math.sqrt(
                            detection.ftcPose.x * detection.ftcPose.x +
                                    detection.ftcPose.y * detection.ftcPose.y
                    );
                    telemetry.addData("test baring", detection.ftcPose.bearing);

                    bearing = detection.ftcPose.bearing;
                    if (distanceToAprilTag >= 67) canShoot = true;
                } catch (Exception e) {
                    telemetry.addData("Error reading detection pose", e.getMessage());
                }
            } else {
                telemetry.addLine("AprilTag detected but pose unavailable.");
            }
        } else {
            // If aprilTagHelper existed but no detections, keep INFF
            if (aprilTagHelper != null) {
                telemetry.addLine("No AprilTag detections");
            } else {
                telemetry.addLine("April Tag not available (helper null)");
            }
        }

        // Safely update pose estimate only if md is available
        if (md != null) {
            try {
                md.updatePoseEstimate();
            } catch (Exception e) {
                telemetry.addData("md.updatePoseEstimate error", e.getMessage());
            }
        } else {
            telemetry.addLine("md (MecanumDrive) is null");
        }

        Pose2d pose = null;
        if (md != null && md.localizer != null) {
            try {
                pose = md.localizer.getPose();
            } catch (Exception e) {
                telemetry.addData("md.localizer.getPose error", e.getMessage());
                pose = null;
            }
        } else {
            telemetry.addLine("md.localizer is null or md is null");
        }

        // Provide safe default values if pose or areaLimiter are not available
        double factor = 0;
        double x = 0;
        double y = 0;
        double heading = 0;
        boolean inTriangle = false;

        if (pose != null) {
            try {
                // original code uses pose.position.x and pose.heading.toDouble()
                // guard for possible nulls inside pose
                if (pose.position != null) {
                    x = pose.position.x;
                    y = pose.position.y;
                } else {
                    telemetry.addLine("pose.position is null; using defaults 0,0");
                }
                try {
                    heading = Math.toDegrees(pose.heading.toDouble());
                    telemetry.addData("heading",heading);
                } catch (Exception e) {
                    // fallback if heading access fails
                    telemetry.addData("Warning", "pose.heading access failed: " + e.getMessage());
                    heading = 0;
                }
            } catch (Exception e) {
                telemetry.addData("Error reading pose fields", e.getMessage());
                x = 0; y = 0; heading = 0;
            }
        } else {
            telemetry.addLine("Pose unavailable; using x=0,y=0,heading=0");
        }

        if (areaLimiter != null) {
            try {
                inTriangle = areaLimiter.inShootingZone(x, y);
            } catch (Exception e) {
                telemetry.addData("areaLimiter.inShootingZone error", e.getMessage());
                inTriangle = false;
            }
        } else {
            telemetry.addLine("areaLimiter is null; cannot check shooting zone");
        }

        double return_val = 0;
        if (util != null) {
            try {
                return_val = util.aimmer(gamepad1.a, bearing, heading, telemetry);
            } catch (Exception e) {
                telemetry.addData("util.aimmer error", e.getMessage());
                return_val = 0;
            }
        } else {
            telemetry.addLine("util is null; aimmer skipped");
        }

        if (aprilTagHelper != null) {
            if (distanceToAprilTag < INFF) {
                telemetry.addData("Distance to April Tag", distanceToAprilTag);
            } else {
                telemetry.addLine("April Tag not detected");
            }
            telemetry.addData("In shooting range", canShoot);
            telemetry.addData("In Shooting Area (Front)", inTriangle);
            if (gamepad1.a && !overwrite) {
                if (util != null) {
                    try {
                        //
                        //telemetry.addLine("auto shooter power engage");
                    } catch (Exception e) {
                        telemetry.addData("util.shooter error", e.getMessage());
                    }
                }
                telemetry.addData("return val", return_val);
                factor = return_val;
                telemetry.addLine("GO SHOOT!!!");
            } else {
                telemetry.addLine("DO NOT SHOOT!!!");
                factor = 0;
                if (util != null) {
                    try {
                        //util.shooter(gamepad1.b, 3000);

                    } catch (Exception e) {
                        telemetry.addData("util.shooter error", e.getMessage());
                    }
                }
            }
        } else {
            telemetry.addLine("camera assist not available, shooter are set to 3000 RPM");
            factor = 0;
        }

        double rawLX = gamepad1.left_stick_x; // strafe (left/right)
        double rawLY = -gamepad1.left_stick_y; // forward/back (invert so up = +)
        double turn = gamepad1.right_stick_x + factor; // rotation

        double limitedLX = rawLX;
        double limitedLY = rawLY;
        if (areaLimiter != null) {
            try {
                double[] limited = areaLimiter.limit(x, y, rawLY, rawLX);
                if (limited != null && limited.length >= 2) {
                    limitedLX = limited[0];
                    limitedLY = limited[1];
                } else {
                    telemetry.addLine("areaLimiter.limit returned null/invalid; using raw sticks");
                }
            } catch (Exception e) {
                telemetry.addData("areaLimiter.limit error", e.getMessage());
            }
            try {
                areaLimiter.hardWall(hardwall);
            } catch (Exception e) {
                telemetry.addData("areaLimiter.hardWall error", e.getMessage());
            }
        } else {
            telemetry.addLine("areaLimiter null; not limiting movement");
        }

        /* remove the comments and put the comments on driveLimited if you want to run a normal teleop(NotLimited)
        right now the mecanum drive got 2 drive system na you can actually delete the other one 'drive()' but it can stay there just for testing and stuff*/

        if (mecanumDriveOwn != null) {
            try {
                mecanumDriveOwn.driveLimited(limitedLX, limitedLY, turn);
            } catch (Exception e) {
                telemetry.addData("mecanumDriveOwn.driveLimited error", e.getMessage());
            }
        } else {
            telemetry.addLine("mecanumDriveOwn is null; cannot drive");
        }
        //mecanumDriveOwn.drive(gamepad1);

        //util.servo_test(hardwareMap, startSequence, telemetry);
        //dummy value: subject to change

        if (util != null) {
            try {
                util.feeder(gamepad1.b);
            } catch (Exception e) {
                telemetry.addData("util.feeder error", e.getMessage());
            }
            try {
                util.lift(gamepad1.x, telemetry);
            } catch (Exception e) {
                telemetry.addData("util.lift error", e.getMessage());
            }
        } else {
            telemetry.addLine("util is null; feeder/lift skipped");
        }

        util.shooter(gamepad1.b, Math.max(2700 , (distanceToAprilTag / 120) * 3000));

        telemetry.addData("Left front motor speed", mecanumDriveOwn.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDriveOwn.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDriveOwn.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDriveOwn.getMotorPower("RBM"));

        telemetry.addData("X", x);
        telemetry.addData("Y", y);

        telemetry.addData("limited X", limitedLX);
        telemetry.addData("limited Y", limitedLY);

        telemetry.addData("shooter power", Math.max(2700 , (distanceToAprilTag / 120) * 3000));
        if (overwrite){
            telemetry.addLine("overwrite engage");
        }
        if (!hardwall){
            telemetry.addLine("hardwall overwrite");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagHelper != null) {
            try {
                aprilTagHelper.shutdown();
            } catch (Exception e) {
                telemetry.addData("aprilTagHelper.shutdown error", e.getMessage());
            }
        }
    }

}