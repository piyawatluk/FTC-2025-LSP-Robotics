package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.teamcode.util.generalUtil;

@Autonomous(name = "blue_auto_front_case", group = "Autonomous")
public final class blue_auto_front_case extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Hardware hw = new Robot_Hardware();
        generalUtil util = new generalUtil(hw);

        Pose2d beginPose = new Pose2d(60, -10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addLine("Blue Auto Front Case Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder segment_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(
                        new Vector2d(50, -10),
                        Math.toRadians(210)
                );
                //.waitSeconds(2);

        TrajectoryActionBuilder segment_2 = drive.actionBuilder(
                        new Pose2d(50, -10, Math.toRadians(210))
                )
                .splineToLinearHeading(
                        new Pose2d(40, -30, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment_2_5 = drive.actionBuilder(new Pose2d(40,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -50));

        TrajectoryActionBuilder segment_2_7 = drive.actionBuilder(new Pose2d(36,-50,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(210)
                );

        TrajectoryActionBuilder segment_3 = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(210))
                )
                .strafeToLinearHeading(
                        new Vector2d(12, -30),
                        Math.toRadians(90)
                );

        TrajectoryActionBuilder segment_3_5 = drive.actionBuilder(new Pose2d(55,10,Math.toRadians(210)))
                .strafeTo(new Vector2d(12, -50));

        TrajectoryActionBuilder segment_3_7 = drive.actionBuilder(new Pose2d(12,-50,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(210)
                );

        TrajectoryActionBuilder end_trajectory = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(210))
                )
                .strafeToLinearHeading(
                        new Vector2d(60, 65),
                        Math.toRadians(180)
                );

        Actions.runBlocking(new SequentialAction(
                segment_1.build(),
                util.fireAction(2800, 0.5, 1.0),
                segment_2.build(),
                new ParallelAction(segment_2_5.build(), util.motorAction(hw.rightBeltDriveMotor, 1,3)),
                segment_2_7.build(),
                util.fireAction(2800, 0.5, 1.0),
                segment_3.build(),
                new ParallelAction(segment_3_5.build(), util.motorAction(hw.rightBeltDriveMotor, 1,3)),
                segment_3_7.build(),
                util.fireAction(2800, 0.5, 1.0),
                end_trajectory.build()
        ));
    }
}
