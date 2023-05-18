package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
Odometry is the concept of using various sensors to accurately track the position
of a robot in regards to the environment around it.
In this case, two dead wheels and encoders along with a gyro sensor are used to
determine X and Y coordinates, as well as heading.
 */
public class OdometrySubsystem {

    private final DcMotor vertical;
    private final DcMotor horizontal;
    private final IMUSubsystem imu;

    // circumference (2πr) over ticks
    private final double TICKS_TO_CM = 2 * Math.PI * (3.5 / 2) / 8192;

    private double x, y, theta;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        imu = new IMUSubsystem(hardwareMap);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");

        vertical.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontal.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        In some cases, you don't want to reset the encoders on initialization, this is because
        the robot will initialize the odometry system between different OpModes.
        If the robot were to move during autonomous period, it would be ideal to save its position on
        the field when transitioning from Auto to TeleOp.
         */
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        x = y = theta = 0;
    }

    public void reset() {
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void updatePosition() {

        x = vertical.getCurrentPosition() * TICKS_TO_CM;

        theta = imu.getAngleDEG();
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        // todo might have to do some extra steps such as /= by 360
        return theta;
    }

}
