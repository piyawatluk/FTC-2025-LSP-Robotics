package org.firstinspires.ftc.teamcode.util;

import  com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorHelp {
    private DistanceSensor distance;

    public void init(HardwareMap hardwareMap){
        distance = hardwareMap.get(DistanceSensor.class, "collision_sensor");
    }

    public double getDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }


}
