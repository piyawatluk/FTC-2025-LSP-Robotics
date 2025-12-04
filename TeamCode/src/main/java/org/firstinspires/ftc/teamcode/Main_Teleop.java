//@ LSP_ROBOTICS_SOFTWARE_DEPARTMENT

/*อิติปิโส ภะคะวา อะระหัง สัมมา สัมพุทโธ วิชชาจะระณะสัมปันโน สุคะโต โลกะวิทู อะนุตตะโร ปุริสะทัมมะสาระถิ สัตถา เทวะมะนุสสานัง พุทโธ ภะคะวาติ

สะวากขาโต ภะคะวะตา ธัมโม สันทิฏฐิโก อะกาลิโก เอหิปัสสิโก โอปะนะยิโก ปัจจัตตัง เวทิตัพโพ วิญญูหิติ

สุปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ อุชุปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ ญายะปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ สามีจิปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ
ยะทิทัง จัตตาริ ปุริสะยุคานิ อัฏฐะ ปุริสะปุคคะลา เอสะ ภะคะวะโต สาวะกะสังโฆ อาหุเนยโย ปาหุเนยโย ทักขิเณยโย อัญชะลีกะระณีโย อะนุตตะรัง ปุญญักเขตตัง โลกัสสาติ

may god be with us
*/

package org.firstinspires.ftc.teamcode;

//essential module import

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AreaLimiter;
import org.firstinspires.ftc.teamcode.util.generalUtil;
import org.firstinspires.ftc.teamcode.util.AprilTag;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    //Hardware declare
    Robot_Hardware hw = new Robot_Hardware();
    generalUtil util = new generalUtil(hw);

    //Drive mode essentials declare
    private MecanumDrive_own mecanumDriveOwn;
    private MecanumDrive md;
    private AreaLimiter areaLimiter;
    private Pose2d pose = null;

    //overwrite logic declare
    private boolean prevBothBumpersG1 = false;
    private boolean prevBothBumpersG2 = false;
    boolean overwrite = false;
    boolean hardwall = true;
    private boolean prevY;
    public double Deg = 0;


    //apriltag declare
    private AprilTag aprilTag;

    @Override
    public void init() {
        //initial startup checkup

        //areaLimiter declare
        areaLimiter = new AreaLimiter(telemetry);

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
            aprilTag = new AprilTag(true, "Webcam 1");
            aprilTag.initialize(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Warning", "AprilTagHelper init failed: " + e.getMessage());
            aprilTag = null;
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addLine("LSP Robotic Senior - Teleop");
        final double infinite_distance = 300;
        double distanceToAprilTag = infinite_distance;

        boolean canShoot = false;

        double shooter_power = 0;

        double rawLX = gamepad1.left_stick_x; // strafe (left/right)
        double rawLY = -gamepad1.left_stick_y; // forward/back (invert so up = +)
        double turn = gamepad1.right_stick_x; // rotation

        double limitedLX = rawLX;
        double limitedLY = rawLY;

        // Provide safe default values if pose or areaLimiter are not available
        double factor;
        double x = 0;
        double y = 0;
        double heading = 0;
        boolean inTriangle = false;

        double bearing = 0; // initial value for bearing
        double manual_RPM = 2800;
        double return_val = 0;

        boolean bothBumpersG2 = gamepad2.left_bumper && gamepad2.right_bumper;
        boolean bothBumpersG1 = gamepad1.left_bumper && gamepad1.right_bumper;

        if (bothBumpersG2 && !prevBothBumpersG2) overwrite = !overwrite;
        if (bothBumpersG1 && !prevBothBumpersG1) hardwall = !hardwall;

        prevBothBumpersG2 = bothBumpersG2;
        prevBothBumpersG1 = bothBumpersG1;

        //Servo swing gate

        if (gamepad1.y && !prevY) {
            if (hw.placeholderServo3.getPosition() < 0.25) {
                hw.placeholderServo3.setPosition(0.5);
            } else {
                hw.placeholderServo3.setPosition(0.0);
            }
        }

        // adjustable for finding the right position
        /*if (gamepad1.y && !prevY && Deg < 1){
            hw.placeholderServo3.setPosition(Deg);
            Deg += 0.1;
        }*/

        prevY = gamepad1.y;

        // Apriltag NPE checking
        List < AprilTagDetection > detections = null;
        if (aprilTag != null) {
            try {
                detections = aprilTag.getDetections();
            } catch (Exception e) {
                telemetry.addData("AprilTagHelper.getDetections error", e.getMessage());
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

                    shooter_power = Math.max(2700, (distanceToAprilTag / 118) * 3000);
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
            if (aprilTag != null) {
                telemetry.addLine("No AprilTag detections");
            } else {
                telemetry.addLine("April Tag not available (helper null)");
            }
        }

        // md NPE checking
        if (md != null) {
            try {
                md.updatePoseEstimate();
            } catch (Exception e) {
                telemetry.addData("md.updatePoseEstimate error", e.getMessage());
            }
        } else {
            telemetry.addLine("md (MecanumDrive) is null");
        }

        // md and localizer NPE checking : subject to change
        if (md != null && md.localizer != null) {
            try {
                pose = md.localizer.getPose();
            } catch (Exception e) {
                telemetry.addData("md.localizer.getPose error", e.getMessage());
            }
        } else {
            telemetry.addLine("md.localizer is null or md is null");
        }

        //pose NPE checking
        if (pose != null) {
            try {
                // original code uses pose.position.x and pose.heading.toDouble()
                // guard for possible nulls inside pose
                x = pose.position.x;
                y = pose.position.y;
                try {
                    heading = Math.toDegrees(pose.heading.toDouble());
                    telemetry.addData("heading", heading);
                } catch (Exception e) {
                    // fallback if heading access fails
                    telemetry.addData("Warning", "pose.heading access failed: " + e.getMessage());
                    heading = 0;
                }
            } catch (Exception e) {
                telemetry.addData("Error reading pose fields", e.getMessage());
                x = 0;
                y = 0;
                heading = 0;
            }
        } else {
            telemetry.addLine("Pose unavailable; using x=0,y=0,heading=0");
        }

        //AreaLimiter NPE checking
        if (areaLimiter != null) {
            try {
                inTriangle = areaLimiter.inShootingZone(x, y);
            } catch (Exception e) {
                telemetry.addData("areaLimiter.inShootingZone error", e.getMessage());
            }
        } else {
            telemetry.addLine("areaLimiter is null; cannot check shooting zone");
        }

        // utility function NPE checking
        if (util != null) {
            try {
                return_val = util.Auto_aim(gamepad1.a, bearing, heading, telemetry);
            } catch (Exception e) {
                telemetry.addData("util.Auto_aim", e.getMessage());
                return_val = 0;
            }
        } else {
            telemetry.addLine("util is null; aimmer skipped");
        }

        //AprilTag NPE checking? : subject to change
        if (aprilTag != null) {
            if (distanceToAprilTag < infinite_distance) {
                telemetry.addData("Distance to April Tag", distanceToAprilTag);
            } else {
                telemetry.addLine("April Tag not detected");
            }
            telemetry.addData("In shooting range", canShoot);
            telemetry.addData("In Shooting Area (Front)", inTriangle);
            if (gamepad1.a && !overwrite) {
                if (util != null) {
                    try {
                        util.shooter(gamepad1.b, shooter_power);
                        telemetry.addLine("auto shooter power engage");
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
            }

        } else {
            telemetry.addLine("camera assist not available, shooter are set to 2800 RPM");
            factor = 0;
            util.shooter(gamepad2.b, manual_RPM);
        }

        //AreaLimiter NPE checking? : subject to change
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

        // mecanumDriveOwn NPE checking
        if (mecanumDriveOwn != null) {
            try {
                mecanumDriveOwn.driveLimited(limitedLX, limitedLY, turn, factor);
            } catch (Exception e) {
                telemetry.addData("mecanumDriveOwn.driveLimited error", e.getMessage());
            }
        } else {
            telemetry.addLine("mecanumDriveOwn is null; cannot drive");
        }

        // utility function NPE checking? : subject to change
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


        // motor speed
        telemetry.addData("Left front motor speed", mecanumDriveOwn.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDriveOwn.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDriveOwn.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDriveOwn.getMotorPower("RBM"));

        // estimate pose
        telemetry.addData("X", x);
        telemetry.addData("Y", y);

        //value return from area limiter
        telemetry.addData("limited X", limitedLX);
        telemetry.addData("limited Y", limitedLY);

        //shooter power
        telemetry.addData("shooter power", shooter_power);


        // telemetry for logic overwrite
        if (overwrite) {
            telemetry.addLine("shooter overwrite engage");
        }

        if (!hardwall) {
            telemetry.addLine("hardwall overwrite engage");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTag != null) {
            try {
                aprilTag.shutdown();
            } catch (Exception e) {
                telemetry.addData("aprilTagHelper.shutdown error", e.getMessage());
            }
        }
    }

}