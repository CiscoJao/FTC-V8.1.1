package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
/*
This is a general PID class that can be used for any application such as for positions and velocities.
Simply initialize an instance of this class with your desired PID constants.
 */

public class PIDController {

    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        timer = new ElapsedTime();
    }

    public double PIDOutput(double current, double target) {
        double error = target - current;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;
        timer.reset();

        return (error * kp) + (integralSum * ki) + (derivative * kd) ;
    }
}
