package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Sequencer;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private Servo servo1;
    private Servo servo2;
    Sequencer Sequence1 = new Sequencer();
    Sequencer Sequence2 = new Sequencer();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Robot_Hardware robotHardware = new Robot_Hardware();
        robotHardware.init(hardwareMap);

        mecanumDrive = new MecanumDrive(robotHardware);
        servo1 = robotHardware.placeholderServo1;
        servo2 = robotHardware.placeholderServo2;
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        Sequence1.add(servo1, 90, 2000, 180, 0);
        Sequence2.add(servo2, 80, 1000, 180, 0);
    }

    @Override
    public void loop() {
        mecanumDrive.drive(gamepad1);
        Sequence1.step();
        Sequence2.step();
    }

    @Override
    public void stop() {}
}
