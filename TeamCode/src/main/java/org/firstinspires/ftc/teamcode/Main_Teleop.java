package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot_Hardware;

@TeleOp(name="Teleop", group="Iterative OpMode")
public class Main_Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot_Hardware robotHardware;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robotHardware = new Robot_Hardware();
        robotHardware.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {}
}
