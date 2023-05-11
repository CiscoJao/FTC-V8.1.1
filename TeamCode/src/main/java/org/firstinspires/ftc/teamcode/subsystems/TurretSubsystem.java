package org.firstinspires.ftc.teamcode.subsystems;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TurretSubsystem {

    private final DcMotor turret;
    private final TouchSensor touch;

    private final double SCALE = 1;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret");

        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    public void turn(double pow) {
        turret.setPower(pow * SCALE);
    }

    public boolean isTouched() {
        return touch.isPressed();
    }
}
