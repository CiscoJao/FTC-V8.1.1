package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelOdometry extends Odometry{

    private final DcMotor vertical;
    private final DcMotor horizontal;
    private final IMUSubsystem imu;

    // constants that define geometry of the robot
    private final double L1 = 1;
    private final double L2 = 1;

    // for tracking position updates
    private double lastVerticalTicks;
    private double lastHorizontalTicks;
    private double currentVerticalTicks;
    private double currentHorizontalTicks;
    private double dVerticalTicks;
    private double dHorizontalTicks;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        imu = new IMUSubsystem(hardwareMap);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");

        // directions are subject to change depending on how you assemble the robot
        vertical.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        In some cases, you don't want to reset the encoders on initialization, this is because
        the robot will initialize the odometry system between different OpModes.
        If the robot were to move during autonomous period, it would be ideal to save its position on
        the field when transitioning from Auto to TeleOp.
         */
        reset();

        /*
        NOTE: x, y, and theta have already been initialized to zero in the constructor of the parent
        class, which will automatically be called when the constructor of this class is called.
        */

        // setting the tuning constants, if no tuning is desired, then leave these as 1
        X_TUNER = 1.01596288;
        Y_TUNER = 1.00755931;
    }

    @Override
    public void reset() {
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void updatePosition() {
        x = vertical.getCurrentPosition() * TICKS_TO_CM;
        y = horizontal.getCurrentPosition() * TICKS_TO_CM;
        theta = imu.getAngleRAD();
//        // finding the total distance (S) encoders have travelled in ticks since the last update
//        currentVerticalTicks = vertical.getCurrentPosition();
//        currentHorizontalTicks = horizontal.getCurrentPosition();
//
//        dVerticalTicks = currentVerticalTicks - lastVerticalTicks;
//        dHorizontalTicks = currentHorizontalTicks - lastHorizontalTicks;
//
//        // finding the changes in position since the last update using the derived movement equations
//        dtheta = imu.getAngularVelocity(); // the change of angle over time is angular velocity
//        dx = (TICKS_TO_CM * dVerticalTicks) + (L1 * dtheta);
//        dy = (TICKS_TO_CM * dHorizontalTicks) + (L2 * dtheta);
//
//        // add the small change in position to the overall position, while also accounting for field orientation
//        // NOTE: remember Java Math uses RADIANS, ensure that all theta values are in RAD
//        x += dx * Math.cos(theta) - dy * Math.sin(theta);
//        y += dx * Math.sin(theta) + dy * Math.cos(theta);
//        theta = imu.getAngleRAD();
//
//        // record the previous encoder positions for the next update
//        lastVerticalTicks = currentVerticalTicks;
//        lastHorizontalTicks = currentHorizontalTicks;
    }
}
