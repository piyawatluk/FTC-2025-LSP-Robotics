package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        class feeder {
            private DcMotorEx motor;
            private CRServo servo1;
            private CRServo servo2;
            public feeder(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(CRServo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            public Action spinUp() {
                return new Action() {
                    private ElapsedTime timer = new ElapsedTime();
                    private boolean started = false;
                    @Override

                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!started) {
                            motor.setPower(1);
                            servo1.setPower(1);
                            servo2.setPower(1);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 3000) {
                            motor.setPower(0);
                            servo1.setPower(0);
                            servo2.setPower(0);
                            return false;
                        }

                        return true;
                    }
                };
            }

        }

        class shooter {
            private DcMotorEx motor;
            private DcMotorEx left_shooter;
            private DcMotorEx right_shooter;
            private CRServo servo1;
            private CRServo servo2;
            public shooter(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                left_shooter = hardwareMap.get(DcMotorEx.class, "lsm");
                right_shooter = hardwareMap.get(DcMotorEx.class, "rsm");
                left_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                left_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                right_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(CRServo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            public Action shoot() {
                return new Action() {
                    private ElapsedTime timer = new ElapsedTime();
                    private boolean started = false;
                    @Override

                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!started) {
                            motor.setPower(1);
                            servo1.setPower(1);
                            servo2.setPower(1);
                            left_shooter.setPower(0.42);
                            right_shooter.setPower(0.42);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 5000) {
                            motor.setPower(0);
                            left_shooter.setPower(0);
                            right_shooter.setPower(0);
                            servo1.setPower(0);
                            servo2.setPower(0);
                            return false;
                        }

                        return true;
                    }
                };
            }
            public Action spinup(){
                return new Action() {
                    private ElapsedTime timer = new ElapsedTime();
                    private boolean started = false;
                    @Override

                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!started) {
                            //motor.setPower(1);
                            //servo1.setPower(1);
                            //servo2.setPower(1);
                            left_shooter.setPower(0.42);
                            right_shooter.setPower(0.42);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 100) {
                            //motor.setPower(0);
                            //servo1.setPower(0);
                            //servo2.setPower(0);
                            return false;
                        }

                        return true;
                    }
                };
            }

        }

        feeder feeder = new feeder(hardwareMap);
        shooter shooter = new shooter(hardwareMap);

        // Wait for PLAY button
        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder segment1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(225));

        TrajectoryActionBuilder segment2 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(230)))
                .splineToLinearHeading(
                        new Pose2d(-15, -25, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment3 = drive.actionBuilder(new Pose2d(-15, -25, Math.toRadians(90)))
                .strafeTo(new Vector2d(-15, -50));

        TrajectoryActionBuilder segment4 = drive.actionBuilder(new Pose2d(-15, -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(220));

        TrajectoryActionBuilder EndTrajectory = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(220)))
                .strafeToLinearHeading(new Vector2d(0, -50), Math.toRadians(180));

        // Main autonomous path
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(segment1.build(),shooter.spinup()),
                //util.fireAction(2800, 0.5, 1.0),
                shooter.shoot(),
                segment2.build(),
                new ParallelAction(segment3.build(),feeder.spinUp()),//.motorAction(hw.rightBeltDriveMotor, 1,3)),
                new ParallelAction(segment4.build(),shooter.spinup()),

                shooter.shoot(),
                //util.fireAction(2800, 0.5, 1.0),
                EndTrajectory.build()
        ));
    }
}
