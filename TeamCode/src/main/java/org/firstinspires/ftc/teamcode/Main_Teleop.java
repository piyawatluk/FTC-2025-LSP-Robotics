package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ServoSequencer;

@TeleOp(name = "Teleop", group = "Iterative OpMode")
public class Main_Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private Servo servo1;
    private Servo servo2;
    ServoSequencer Sequence1 = new ServoSequencer();
    ServoSequencer Sequence2 = new ServoSequencer();

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
        Sequence1.addAction(servo1, 90, 2000, 180, 0);
        Sequence2.addAction(servo2, 80, 1000, 180, 0);
    }

    @Override
    public void loop() {
        mecanumDrive.drive(gamepad1);
        Sequence1.stepSequence();
        Sequence2.stepSequence();
    }

    @Override
    public void stop() {}
}
