package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem {

    private final Rev2mDistanceSensor dist;

    public SensorSubsystem(HardwareMap hardwareMap) {
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
    }

    public double getDistanceCM() {
        return dist.getDistance(DistanceUnit.CM);
    }
}
