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
    private final PIDController pidController;

    private final double SCALE = 0.3;
    public static final double DEG_TO_TICKS = 780.0 / 360.0; // degrees to ticks ratio

    // TODO DELETE LATER
    public int targetPos = 0; // public variable for now for testing purposes, delete later

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret");

        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CWLimit = hardwareMap.get(TouchSensor.class, "cwlimit");
        CCWLimit = hardwareMap.get(TouchSensor.class, "ccwlimit");

        pidController = new PIDController(0.001, 0, 0);
    }

    // general turn method that turns by a power
    public void turn(double pow) {
        if ((pow < 0 && CCWLimit.isPressed()) || (pow > 0 && CWLimit.isPressed()))
            stop();
        else
            turret.setPower(pow * SCALE);
    }

    public void turnPID(int current, int target) {
        turret.setPower(pidController.PIDOutput(current, target));
    }

    // todo DELETE LATER
    // turn method that turns by the angle given
    public void turnByAngle(double deg) {
        targetPos = turret.getCurrentPosition() + (int) Math.round(deg * DEG_TO_TICKS);

        // set the target and power the motor
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (turret.getCurrentPosition() < targetPos + 1 || turret.getCurrentPosition() > targetPos - 1) {
            // stop the motors early if one of the limits are hit
            if ((CCWLimit.isPressed() && deg < 0) || (CWLimit.isPressed() && deg > 0))
              break;

            turret.setPower(pidController.PIDOutput(turret.getCurrentPosition(), targetPos));
        }

        stop();
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
