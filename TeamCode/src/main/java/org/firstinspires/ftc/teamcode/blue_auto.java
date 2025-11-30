package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Blue Auto", group = "Autonomous")
public final class blue_auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Starting pose (this must match your actual robot location on field)
        Pose2d beginPose = new Pose2d(60, -10, Math.toRadians(180));

        // Create the Road Runner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // VERY IMPORTANT: Set pose estimate
        //drive.setPoseEstimate(beginPose);

        telemetry.addLine("Blue Auto Ready");
        telemetry.update();

        // Wait for PLAY button
        waitForStart();

        if (isStopRequested()) return;

        // Main autonomous path
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(50, -10), Math.toRadians(210))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(36,-30,Math.toRadians(90)), Math.toRadians(-90))
                        .strafeTo(new Vector2d(36,-50))
                        .strafeToLinearHeading(new Vector2d(55,-10),Math.toRadians(210))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(12,-30),Math.toRadians(90))
                        .strafeTo(new Vector2d(12,-50))
                        .strafeToLinearHeading(new Vector2d(55,-10),Math.toRadians(210))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(60,65),Math.toRadians(180))
                        .build()
        );
    }
}