package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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
            private Servo servo2;
            public feeder(HardwareMap hardwareMap) {
                motor = hardwareMap.get(DcMotorEx.class, "rbdm");
                servo1 = hardwareMap.get(CRServo.class, "tbd_0");
                servo2 = hardwareMap.get(Servo.class,"tbd_1");
                servo1.setDirection(DcMotorSimple.Direction.FORWARD);
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
                servo1.setDirection(DcMotorSimple.Direction.FORWARD);
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
                servo1.setDirection(DcMotorSimple.Direction.FORWARD);
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
                            left_shooter.setPower(0.41);
                            right_shooter.setPower(0.41);
                            timer.reset();
                            started = true;
                        }

                        if (timer.milliseconds() >= 4000) {
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
                            left_shooter.setPower(0.41);
                            right_shooter.setPower(0.41);
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

        // Wait for PLAY button
        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder segment1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-15.5, -15.5), Math.toRadians(225));

        TrajectoryActionBuilder segment2 = drive.actionBuilder(new Pose2d(-15.5, -15.5, Math.toRadians(230)))
                .splineToLinearHeading(
                        new Pose2d(-3.5, -20, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment3 = drive.actionBuilder(new Pose2d(-3.5, -20, Math.toRadians(90)))
                .strafeTo(new Vector2d(-3.5, -53), new TranslationalVelConstraint(15));

        TrajectoryActionBuilder segment4 = drive.actionBuilder(new Pose2d(-3.5, -53, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-15, -15), Math.toRadians(212.5));

        TrajectoryActionBuilder segment5 = drive.actionBuilder(new Pose2d(-15,-15,Math.toRadians(90)))
                .splineToLinearHeading(
                        new Pose2d(27, -25, Math.toRadians(90)),
                        Math.toRadians(-90)
                );

        TrajectoryActionBuilder segment6 = drive.actionBuilder(new Pose2d(27,-25,Math.toRadians(90)))
                .splineToLinearHeading(
                        new Pose2d(27, -60, Math.toRadians(90)),
                        Math.toRadians(-90), new TranslationalVelConstraint(15)
                );

        TrajectoryActionBuilder EndTrajectory = drive.actionBuilder(new Pose2d(-15, -15, Math.toRadians(212.5)))
                .strafeToLinearHeading(new Vector2d(0, -65), Math.toRadians(180));

        // Main autonomous path
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(segment1.build(),shooter.spinup(),gate.open()),
                //util.fireAction(2800, 0.5, 1.0),
                shooter.shoot(),
                segment2.build(),
                new ParallelAction(segment3.build(),feeder.spinUp()),//.motorAction(hw.rightBeltDriveMotor, 1,3)),
                new ParallelAction(segment4.build(),shooter.spinup()),
                shooter.shoot(),
                segment5.build(),
                new ParallelAction(segment6.build(),feeder_2.spinUp()),
                segment4.build(),
                shooter.shoot()
                //util.fireAction(2800, 0.5, 1.0),
                //EndTrajectory.build()
        ));
    }
}
