package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TwoWheelOdometry extends Odometry{

    private final DcMotor vertical;
    private final DcMotor horizontal;
    private final IMUSubsystem imu;

    // constants that define geometry of the robot
    private final double L1 = 20.15618755; // distance of the vertical from the center of rotation
    private final double L2 = 17.87437649; // distance of the horizontal from the center of rotation

    // for tracking position updates
    private double lastVerticalTicks;
    private double lastHorizontalTicks;
    private double currVerticalTicks;
    private double currHorizontalTicks;
    private double dVerticalTicks;
    private double dHorizontalTicks;

    // todo delete later
    private ElapsedTime timer;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        imu = new IMUSubsystem(hardwareMap);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");

        // directions are subject to change depending on how you assemble the robot
        vertical.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        NOTE: x, y, and theta have already been initialized to zero in the constructor of the parent
        class, which will automatically be called when the constructor of this class is called.
        */

        // setting the tuning constants, if no tuning is desired, then leave these as 1
        X_TUNER = 1;
        Y_TUNER = 1;

        // todo delete later
        timer = new ElapsedTime();
    }

    @Override
    public void reset() {
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        x = y = theta = 0;
    }

    @Override
    public void updatePosition() {
//        x = vertical.getCurrentPosition() * TICKS_TO_CM;
//        y = horizontal.getCurrentPosition() * TICKS_TO_CM;
//        theta = imu.getAngleRAD();

        timer.reset();

        currVerticalTicks = vertical.getCurrentPosition();
        currHorizontalTicks = horizontal.getCurrentPosition();

        dVerticalTicks = currVerticalTicks - lastVerticalTicks;
        dHorizontalTicks = currHorizontalTicks - lastHorizontalTicks;

        x = dVerticalTicks * TICKS_TO_CM / timer.seconds();
        y = dHorizontalTicks * TICKS_TO_CM / timer.seconds();
        theta = imu.getAngularVelocity();

        lastVerticalTicks = currVerticalTicks;
        lastHorizontalTicks = currHorizontalTicks;
    }

    // DOES NOT ACCOUNT FOR WHEN THE ROBOT IS STATIONARY BUT ROTATING
    public void updatePositionOld() {

        // finding the total distance (S) encoders have travelled in ticks since the last update
        currVerticalTicks = vertical.getCurrentPosition();
        currHorizontalTicks = horizontal.getCurrentPosition();

        dVerticalTicks = currVerticalTicks - lastVerticalTicks;
        dHorizontalTicks = currHorizontalTicks - lastHorizontalTicks;

        // finding the changes in position since the last update using the derived movement equations
        dtheta = imu.getAngularVelocity(); // the change of angle over time is angular velocity
        dx = (TICKS_TO_CM * dVerticalTicks) + (L1 * dtheta);
        dy = (TICKS_TO_CM * dHorizontalTicks) + (L2 * dtheta);

        // add the small change in position to the overall position, while also accounting for field orientation
        // NOTE: remember Java Math uses RADIANS, ensure that all theta values are in RAD
        x += dx * Math.cos(theta) - dy * Math.sin(theta);
        y += dx * Math.sin(theta) + dy * Math.cos(theta);
        theta = imu.getAngleRAD();

        // record the previous encoder positions for the next update
        lastVerticalTicks = currVerticalTicks;
        lastHorizontalTicks = currHorizontalTicks;
    }
}
