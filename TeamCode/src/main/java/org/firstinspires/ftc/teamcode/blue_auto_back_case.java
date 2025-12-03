package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.generalUtil;

@Autonomous(name = "blue_auto_back_case", group = "Autonomous")
public final class blue_auto_back_case extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Hardware hw = new Robot_Hardware();
        generalUtil util = new generalUtil(hw);

        // Starting pose (this must match your actual robot location on field)
        Pose2d beginPose = new Pose2d(-50, -50, Math.toRadians(230));

        // Create the Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // VERY IMPORTANT: Set pose estimate
        //drive.setPoseEstimate(beginPose);

        telemetry.addLine("Blue Auto Ready");
        telemetry.update();

        // Wait for PLAY button
        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder segment1 = drive.actionBuilder(beginPose
        )
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(230));

        TrajectoryActionBuilder segment2 = drive.actionBuilder(new Pose2d(0,0,230))
                .splineToLinearHeading(new Pose2d(-15,-25,Math.toRadians(90)), Math.toRadians(-90));

        TrajectoryActionBuilder segment3 = drive.actionBuilder(new Pose2d(-15,-25, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-15,-50));

        TrajectoryActionBuilder segment4 = drive.actionBuilder(new Pose2d(-5,-50,Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(220));

        TrajectoryActionBuilder EndTrajectory = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(220)))
                .strafeToLinearHeading(new Vector2d(5,-50), Math.toRadians(270));

        // Main autonomous path
        Actions.runBlocking(new SequentialAction(
                segment1.build(),
                //util.fireAction(2800, 0.5, 1.0),
                segment2.build(),
                new ParallelAction(segment3.build()),//.motorAction(hw.rightBeltDriveMotor, 1,3)),
                segment4.build(),
                //util.fireAction(2800, 0.5, 1.0),
                EndTrajectory.build()
        ));
    }
}
