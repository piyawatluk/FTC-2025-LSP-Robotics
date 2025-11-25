package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.generalUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")

public class blue_auto extends LinearOpMode {
    private final Robot_Hardware hardware;
    Robot_Hardware hw = new Robot_Hardware();

    public blue_auto(Robot_Hardware hw){
        this.hardware = hw;
    }
    generalUtil util = new generalUtil(hw);

    public class Shooter {
        //private DcMotorEx motor;

        //public Shooter(HardwareMap hardwareMap) {
            //motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        //}

        public class SpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    util.shooter(true,3000);
                    initialized = true;
                }

                //double vel = motor.getVelocity();
                //packet.put("shooterVelocity", vel);
                //return vel < 10_000.0;
                return false;
            }
        }

        public Action spinUp() {
            return new SpinUp();
        }
    }


    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter();
        Pose2d initialPose = new Pose2d(55, -10, Math.toRadians(200));
        Pose2d leaving = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Claw claw = new Claw(hardwareMap);
        //Lift lift = new Lift();

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(36,-30,Math.toRadians(90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(36,-50))
                .strafeToLinearHeading(new Vector2d(55,-10),Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(12,-30),Math.toRadians(90))
                .strafeTo(new Vector2d(12,-50))
                .strafeToLinearHeading(new Vector2d(55,-10),Math.toRadians(200));

        TrajectoryActionBuilder mov1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(36,-30,Math.toRadians(90)), Math.toRadians(-90));


        TrajectoryActionBuilder mov2 = drive.actionBuilder(leaving)
                .splineToLinearHeading(new Pose2d(10,0,0),0);


        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, 55),Math.toRadians(180))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        //int startPosition = visionOutputPosition;
        //telemetry.addData("Starting Position", startPosition);
        //telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action p1;
        p1 = mov1.build();

        Actions.runBlocking(
                new SequentialAction(
                        p1
                )
        );
    }
}