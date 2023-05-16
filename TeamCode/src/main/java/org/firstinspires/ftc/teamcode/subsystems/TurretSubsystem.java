package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class TurretSubsystem {

    private final DcMotor turret;
    private final TouchSensor CWLimit;
    private final TouchSensor CCWLimit;
    private final PIDController followPID;

    private final double SCALE = 0.3;
    public static final double DEG_TO_TICKS = 780.0 / 360.0; // degrees to ticks ratio

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret");

        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // disables the built in PID controller in the motor
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CWLimit = hardwareMap.get(TouchSensor.class, "cwlimit");
        CCWLimit = hardwareMap.get(TouchSensor.class, "ccwlimit");

        // this PID controller is tuned for following the yellow poles
        followPID = new PIDController(0.002, 0.00, 0.00001);

        // this PID controller is tuned for moving the turret in 90 degree turns
        // todo add PID controller
    }

    // general turn method that turns by a power
    public void turn(double pow) {
        if ((pow < 0 && CCWLimit.isPressed()) || (pow > 0 && CWLimit.isPressed()))
            stop();
        else
            turret.setPower(pow * SCALE);
    }

    public void followPID(int current, int target) {
        turret.setPower(followPID.PIDOutput(current, target));
    }
    public void stop() {
        turret.setPower(0);
    }

    public boolean clockWiseLimit() {
        return CWLimit.isPressed();
    }

    public boolean counterClockWiseLimit() {
        return CCWLimit.isPressed();
    }

    public double getMotorPower() {
        return turret.getPower();
    }

    public int getPosition() {
        return turret.getCurrentPosition();
    }
}
