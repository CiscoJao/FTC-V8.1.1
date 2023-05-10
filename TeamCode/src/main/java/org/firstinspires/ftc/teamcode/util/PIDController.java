package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems_legacy.PIDSubsystem;

public class PIDController {

    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}
