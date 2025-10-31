package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.generalUtil;
import org.firstinspires.ftc.teamcode.util.Sequencer;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private Servo servo1;
    private Servo servo2;

    private Sequencer sequence1 = new Sequencer();
    private Sequencer sequence2 = new Sequencer();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Robot_Hardware robotHardware = new Robot_Hardware();
        robotHardware.init(hardwareMap, telemetry);

        mecanumDrive = new MecanumDrive(robotHardware);
        servo1 = robotHardware.placeholderServo1;
        servo2 = robotHardware.placeholderServo2;
    }

    @Override
    public void start() {
        sequence1.add(servo1, 0.5, 2000);
        sequence1.add(servo1, 0.2, 600, true);
        sequence2.add(servo2, 0.6, 100);
        sequence2.add(830);

        runtime.reset();
    }

    @Override
    public void loop() {
        mecanumDrive.drive(gamepad1);

        telemetry.addLine("LSP Robotic Senior - Teleop");
        telemetry.addLine("___________________________");
        telemetry.addData("Left front motor speed", mecanumDrive.getMotorPower("LFM"));
        telemetry.addData("Right front motor speed", mecanumDrive.getMotorPower("RFM"));
        telemetry.addData("Left rear motor speed", mecanumDrive.getMotorPower("LBM"));
        telemetry.addData("Right rear motor speed", mecanumDrive.getMotorPower("RBM"));

        //sequence1.step();
        //if (sequence1.actionCounter > 1) {
        //    sequence2.step();
        //}

        generalUtil.servo_test(gamepad1.a);
        telemetry.update();
    }

    @Override
    public void stop() {}
}
