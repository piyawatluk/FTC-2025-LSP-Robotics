package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "blue_auto_back_case", group = "Autonomous")
public final class blue_auto_back_case extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

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

        // Main autonomous path
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(230))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(-5,-25,Math.toRadians(90)), Math.toRadians(-90))
                        .strafeTo(new Vector2d(-5,-50))
                        .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(220))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(5,-50), Math.toRadians(270))
                        .build()
        );
    }
}
