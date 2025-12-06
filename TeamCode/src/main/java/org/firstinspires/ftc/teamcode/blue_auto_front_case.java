package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.teamcode.util.Sequencer;
import org.firstinspires.ftc.teamcode.util.generalUtil;

import java.security.CryptoPrimitive;
import java.util.Arrays;

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
            private CRServo servo1;
            private Servo servo2;
            public feeder(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(Servo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(Servo.Direction.REVERSE);
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
                            servo2.setPosition(0.5);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 2300) {
                            motor.setPower(0);
                            servo1.setPower(0);
                            servo2.setPosition(0.5);
                            //servo2.setPower(0);
                            return false;
                        }

                        return true;
                    }
                };
            }

        }

        class feeder_2 {
            private DcMotorEx motor;
            private CRServo servo1;
            private Servo servo2;
            public feeder_2(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(Servo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(Servo.Direction.REVERSE);
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
                            servo2.setPosition(0.5);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 2700) {
                            motor.setPower(0);
                            servo1.setPower(0);
                            servo2.setPosition(0.5);
                            //servo2.setPower(0);
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
            private Servo servo2;
            public shooter(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                left_shooter = hardwareMap.get(DcMotorEx.class, "lsm");
                right_shooter = hardwareMap.get(DcMotorEx.class, "rsm");
                left_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
                left_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_shooter.setDirection(DcMotorSimple.Direction.FORWARD);
                right_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(Servo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(Servo.Direction.REVERSE);
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
                            servo2.setPosition(1);
                            left_shooter.setPower(0.45);
                            right_shooter.setPower(0.45);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 5000) {
                            motor.setPower(0);
                            servo1.setPower(0);
                            servo2.setPosition(0.5);
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
                            servo2.setPosition(0.5);
                            left_shooter.setPower(0.45);
                            right_shooter.setPower(0.45);
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

        class gate {
            private Servo servo2;
            public gate(HardwareMap hardwareMap){
                servo2 = hardwareMap.get(Servo.class,"tbd_1");
                servo2.setDirection(Servo.Direction.REVERSE);
            }
            public Action open(){
                return new Action() {
                    private ElapsedTime timer = new ElapsedTime();
                    private boolean started = false;
                    @Override

                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!started) {
                            //motor.setPower(1);
                            //servo1.setPower(1);
                            servo2.setPosition(1);
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
        feeder_2 feeder_2 = new feeder_2(hardwareMap);
        shooter shooter = new shooter(hardwareMap);
        gate gate = new gate(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder segment_1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(198)
                );

        TrajectoryActionBuilder segment_2 = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(198))
                )
                .splineToLinearHeading(
                        new Pose2d(38, -30, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment_2_5 = drive.actionBuilder(new Pose2d(38,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(38, -69),new TranslationalVelConstraint(20));


        TrajectoryActionBuilder segment_2_7 = drive.actionBuilder(new Pose2d(40,-69,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(198)
                );

        TrajectoryActionBuilder segment_3 = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(198))
                )
                .strafeToLinearHeading(
                        new Vector2d(16, -30),
                        Math.toRadians(90)
                );
                //.waitSeconds(2);

        TrajectoryActionBuilder segment_3_5 = drive.actionBuilder(new Pose2d(16,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(16, -69));
                //.waitSeconds(2);

        TrajectoryActionBuilder segment_3_7 = drive.actionBuilder(new Pose2d(16,-69,Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(55, -10),
                        Math.toRadians(198)
                );

        TrajectoryActionBuilder end_trajectory = drive.actionBuilder(
                        new Pose2d(55, -10, Math.toRadians(198))
                )
                .strafeToLinearHeading(
                        new Vector2d(55, 70),
                        Math.toRadians(180)
                );
                //.waitSeconds(2);

        Actions.runBlocking(new SequentialAction(
                shooter.spinup(),
                segment_1.build(),
                //shooter.spinup(),
                shooter.shoot(),
                segment_2.build(),
                new ParallelAction(segment_2_5.build(),feeder.spinUp()),
                new ParallelAction(segment_2_7.build(),shooter.spinup()),
                shooter.shoot(),
                segment_3.build(),
                new ParallelAction(segment_3_5.build(),feeder.spinUp()),
                new ParallelAction(segment_3_7.build(),shooter.spinup()),
                shooter.shoot(),
                end_trajectory.build()

        )); //hope it works
    }
}