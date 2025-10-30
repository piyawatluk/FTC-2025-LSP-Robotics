package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.generalUtil;

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
        robotHardware.init(hardwareMap,telemetry);

        mecanumDrive = new MecanumDrive(robotHardware);
        servo1 = robotHardware.placeholderServo1;
        servo2 = robotHardware.placeholderServo2;
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        Sequence1.add(servo1, 0.5, 2000);
        Sequence1.add(servo1, 0.2, 600, true);
        Sequence2.add(servo2, 0.6, 100);
        Sequence2.add(830);

        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addLine("LSP robotic senior - Teleop");
        mecanumDrive.drive(gamepad1);
        //Sequence1.step();
        //if (Sequence1.actionCounter > 1) {
            //Sequence2.step();
        //}
        generalUtil.servo_test(gamepad1.a);



    }

    @Override
    public void stop() {}
}
