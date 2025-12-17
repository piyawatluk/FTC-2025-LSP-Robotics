//@ LSP_ROBOTICS_SOFTWARE_DEPARTMENT

/*อิติปิโส ภะคะวา อะระหัง สัมมา สัมพุทโธ วิชชาจะระณะสัมปันโน สุคะโต โลกะวิทู อะนุตตะโร ปุริสะทัมมะสาระถิ สัตถา เทวะมะนุสสานัง พุทโธ ภะคะวาติ

สะวากขาโต ภะคะวะตา ธัมโม สันทิฏฐิโก อะกาลิโก เอหิปัสสิโก โอปะนะยิโก ปัจจัตตัง เวทิตัพโพ วิญญูหิติ

สุปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ อุชุปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ ญายะปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ สามีจิปะฏิปันโน ภะคะวะโต สาวะกะสังโฆ
ยะทิทัง จัตตาริ ปุริสะยุคานิ อัฏฐะ ปุริสะปุคคะลา เอสะ ภะคะวะโต สาวะกะสังโฆ อาหุเนยโย ปาหุเนยโย ทักขิเณยโย อัญชะลีกะระณีโย อะนุตตะรัง ปุญญักเขตตัง โลกัสสาติ

may god be with us amen!
*/

package org.firstinspires.ftc.teamcode;

//essential module import

import android.annotation.SuppressLint;

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
    boolean prevSquare = false;
    boolean overwrite = false;
    boolean hardwall = false;

    int dPadCount = 0;
    int dPadCount_sec = 0;
    boolean wasDpadUpPressed = false;
    boolean wasDpadDownPressed = false;
    boolean wasDpadLeftPressed = false;
    boolean wasDpadRightPressed = false;
    boolean prevY = false;

    private boolean prevA = false;
    boolean autoaim = false;
    //boolean power = false;


    //apriltag declare
    private AprilTag aprilTag;

    private long lastLoopTimeNs = 0;
    private double avgLoopMs = 0;

    boolean teamblue = true; //blue on default, red if toggled
    double manual_RPM;


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


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {


        telemetry.addLine("LSP Robotic Senior - Teleop");

        final double infinite_distance = 300;
        double distanceToAprilTag = infinite_distance;

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

        double bearing = 0; // initial value for bearing

        double return_val = 0;

        boolean bothBumpersG2 = gamepad2.left_bumper && gamepad2.right_bumper;
        boolean bothBumpersG1 = gamepad1.left_bumper && gamepad1.right_bumper;

        //if (gamepad1.y && prevY){
            //power = !power;
        //}
        //prevY = gamepad1.y;

        //if (power){
            //manual_RPM = 2500;
        //}
        //else {
            //manual_RPM = 2900;
        //}

        if (bothBumpersG2 && !prevBothBumpersG2) overwrite = !overwrite;
        if (bothBumpersG1 && !prevBothBumpersG1) hardwall = !hardwall;

        prevBothBumpersG2 = bothBumpersG2;
        prevBothBumpersG1 = bothBumpersG1;

        if(gamepad2.x && !prevSquare) teamblue = !teamblue;

        prevSquare = gamepad2.square;

        if(teamblue) telemetry.addLine("Team Blue");
        else telemetry.addLine("Team Red");

        if (gamepad1.dpad_up && !wasDpadUpPressed) {
            dPadCount = Math.min(dPadCount + 1, 1);
        } else if (gamepad1.dpad_down && !wasDpadDownPressed) {
            dPadCount = Math.max(dPadCount - 1, 0);
        }

        wasDpadUpPressed = gamepad1.dpad_up;
        wasDpadDownPressed = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !wasDpadRightPressed){
            dPadCount_sec = Math.min(dPadCount_sec + 1,5);
        } else if (gamepad1.dpad_left && ! wasDpadLeftPressed) {
            dPadCount_sec = Math.max(dPadCount_sec - 1,0);
        }

        wasDpadLeftPressed = gamepad1.dpad_left;
        wasDpadRightPressed = gamepad1.dpad_right;

        if (dPadCount_sec == 0){
            manual_RPM = 2300;
            telemetry.addLine("2300");
        } else if (dPadCount_sec == 1) {
            manual_RPM = 2500;
            telemetry.addLine("2500");
        } else if (dPadCount_sec == 2) {
            manual_RPM = 2700;
            telemetry.addLine("2700");
        } else if (dPadCount_sec == 3) {
            manual_RPM = 2900;
            telemetry.addLine("2900");
        } else if (dPadCount_sec == 4) {
            manual_RPM = 3100;
            telemetry.addLine("3100");
        } else if (dPadCount_sec == 5) {
            manual_RPM = 3300;
            telemetry.addLine("3300");
        }

        if (gamepad1.a && !prevA) {
            autoaim = !autoaim;
        }

        prevA = gamepad1.a;

        if (!autoaim) {
            if (aprilTag != null) aprilTag.shutdown();
        } else {
            if (aprilTag != null) aprilTag.initialize(hardwareMap);
        }


        // Apriltag NPE checking
        List<AprilTagDetection> detections = null;
        if (aprilTag != null) {
            try {
                detections = aprilTag.getDetections();
            } catch (Exception e) {
                telemetry.addData("AprilTagHelper.getDetections error", e.getMessage());
            }
        } else {
            telemetry.addLine("aprilTagHelper is null; skipping vision");
        }
        int targetTagId = teamblue ? 20 : 24;
        AprilTagDetection targetDetection = null;
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                if (detection != null && detection.id == targetTagId) {
                    targetDetection = detection;
                    break; // stop once the correct team tag is found
                }
            }
            if (targetDetection != null && targetDetection .ftcPose != null) {
                try {
                    distanceToAprilTag = Math.sqrt(
                            targetDetection .ftcPose.x * targetDetection .ftcPose.x +
                                    targetDetection .ftcPose.y * targetDetection .ftcPose.y
                    );

                    //shooter_power = Math.max(2700, (distanceToAprilTag / 118) * 3000); //tbd
                    bearing = targetDetection .ftcPose.bearing;
                    telemetry.addData("test baring", bearing);


                } catch (Exception e) {
                    telemetry.addData("Error reading detection pose", e.getMessage());
                }
                try {
                    telemetry.addData("Team AprilTag ID : ", targetDetection.id);
                } catch (Exception e) {
                    telemetry.addLine("Cannot find team's AprilTag");
                }
            } else {
                telemetry.addLine("AprilTag detected but pose unavailable.");
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
            } catch (Exception e) {
                telemetry.addData("Error reading pose fields", e.getMessage());
                x = 0;
                y = 0;
            }
        } else {
            telemetry.addLine("Pose unavailable; using x=0,y=0,heading=0");
        }

        //AreaLimiter NPE checking
        if (areaLimiter != null) {
            try {
                //telemetry.addLine("Telemetry not null");
            } catch (Exception e) {
                telemetry.addData("areaLimiter.inShootingZone error", e.getMessage());
            }
        } else {
            telemetry.addLine("areaLimiter is null; cannot check shooting zone");
        }

        // utility function NPE checking
        if (util != null) {
            try {
                return_val = util.Auto_aim(autoaim, bearing, telemetry);
            } catch (Exception e) {
                telemetry.addData("util.Auto_aim", e.getMessage());
                return_val = 0;
            }
        } else {
            telemetry.addLine("util is null; aimer skipped");
        }

        //AprilTag NPE checking? : subject to change
        if (aprilTag != null) {
            if (distanceToAprilTag < infinite_distance) {
                telemetry.addData("Distance to April Tag", distanceToAprilTag);
            }
            if (autoaim && !overwrite) {
                if (util != null) {
                    try {
                        util.shooter(gamepad1.left_bumper, manual_RPM);
                        //util.gate(gamepad1.left_bumper);
                    } catch (Exception e) {
                        telemetry.addData("util.shooter error", e.getMessage());
                    }
                }
                factor = return_val;
            } else {
                factor = 0;
            }

        } else {
            //telemetry.addLine("camera assist not available, shooter are set to 2800 RPM");
            factor = 0;
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
        if (util != null ) {
            try {
                util.feeder(gamepad1.b);
            } catch (Exception e) {
                telemetry.addData("util.feeder error", e.getMessage());
            }
            try {
                util.lift(telemetry, dPadCount);
            } catch (Exception e) {
                telemetry.addData("util.lift error", e.getMessage());
            }
        } else {
            telemetry.addLine("util is null; feeder/lift skipped");
        }

        //shooter power
        //telemetry.addData("Shooter power :", shooter_power);


        // telemetry for logic overwrite
        if (overwrite) {
            telemetry.addLine("Shooter Overwrite Engaged");

        }

        if (!hardwall) {
            telemetry.addLine("Hardwall Overwrite Engaged");
        }

        //if (!power){
            //telemetry.addLine("power at 3000 rpm");
        //} else {
            //telemetry.addLine("power at 2800 rpm");
        //}

        long now = System.nanoTime();

        if (lastLoopTimeNs != 0) {
            double loopMs = (now - lastLoopTimeNs) / 1e6;

            // Smooth average (prevents flicker)
            avgLoopMs = 0.9 * avgLoopMs + 0.1 * loopMs;

            telemetry.addData("Loop Time (ms)", String.format("%.2f", loopMs));
            telemetry.addData("Avg Loop (ms)", String.format("%.2f", avgLoopMs));

            if (avgLoopMs > 50) telemetry.addLine("CPU HIGH LOAD");
            if (avgLoopMs > 80) telemetry.addLine("CPU OVERLOAD");
            if (avgLoopMs > 120) telemetry.addLine("CRASH IMMINENT");
        }

        lastLoopTimeNs = now;
        telemetry.update();


    }

    @Override
    public void stop() {
        try {
            if (aprilTag != null) {
                aprilTag.shutdown();
                aprilTag = null;
            }

            mecanumDriveOwn = null;
            md = null;
            areaLimiter = null;
            util = null;
            hw = null;

            //System.gc(); // Encourage memory cleanup on FTC devices
        } catch (Exception e) {
            telemetry.addData("Stop cleanup error", e.getMessage());
        }
    }


}