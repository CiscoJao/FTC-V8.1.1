package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDSubsystem {

    private DigitalChannel red;
    private DigitalChannel green;

    public LEDSubsystem(HardwareMap hardwareMap) {
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");

        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void greenLight() {
        red.setState(false);
        green.setState(true);
    }

    public void redLight() {
        red.setState(true);
        green.setState(false);
    }
}
