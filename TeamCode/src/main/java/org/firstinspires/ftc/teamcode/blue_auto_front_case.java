package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.generalUtil;

@Autonomous(name = "blue_auto_front_case", group = "Autonomous")
public final class blue_auto_front_case extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // create hw, then initialize it with the OpMode hardwareMap


        // only create util after hardware has been initialized
        //generalUtil util = new generalUtil(hw);

        Pose2d beginPose = new Pose2d(60, -10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addLine("Blue Auto Front Case Ready");
        telemetry.update();

        Sequencer sequencer = new Sequencer();

        class feeder {
            private DcMotorEx motor;

            public feeder(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "rbdm");
            }

            public Action spinUp() {
                return new Action() {
                    private ElapsedTime timer = new ElapsedTime();
                    private boolean started = false;
                    @Override

                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!started) {
                            motor.setPower(1);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 2000) {
                            motor.setPower(0);
                            return false;
                        }

                        return true;
                    }
                };
            }

        }

        feeder feeder = new feeder(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder segment_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(
                        new Vector2d(50, -10),
                        Math.toRadians(200)
                )
                .waitSeconds(2);

        TrajectoryActionBuilder segment_2 = drive.actionBuilder(
                        new Pose2d(50, -10, Math.toRadians(200))
                )
                .splineToLinearHeading(
                        new Pose2d(36, -30, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment_2_5 = drive.actionBuilder(new Pose2d(36,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -50))
                .waitSeconds(2);

        TrajectoryActionBuilder segment_2_7 = drive.actionBuilder(new Pose2d(36,-50,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(200)
                )
                .waitSeconds(2);

        TrajectoryActionBuilder segment_3 = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(200))
                )
                .strafeToLinearHeading(
                        new Vector2d(12, -30),
                        Math.toRadians(90)
                )
                .waitSeconds(2);

        TrajectoryActionBuilder segment_3_5 = drive.actionBuilder(new Pose2d(12,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(12, -50))
                .waitSeconds(2);

        TrajectoryActionBuilder segment_3_7 = drive.actionBuilder(new Pose2d(12,-50,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(200)
                )
                .waitSeconds(2);

        TrajectoryActionBuilder end_trajectory = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(200))
                )
                .strafeToLinearHeading(
                        new Vector2d(55, 55),
                        Math.toRadians(180)
                )
                .waitSeconds(2);

        Actions.runBlocking(new SequentialAction(
                segment_1.build(),
                segment_2.build(),
                new ParallelAction(segment_2_5.build(),feeder.spinUp()),
                segment_2_7.build(),
                segment_3.build(),
                segment_3_5.build(),
                segment_3_7.build(),
                end_trajectory.build()

        )); //hope it works
    }
}