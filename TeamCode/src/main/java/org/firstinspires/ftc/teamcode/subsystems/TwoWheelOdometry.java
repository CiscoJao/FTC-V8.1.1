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
    private final double L1 = 1; // distance of the vertical from the center of rotation
    private final double L2 = 1; // distance of the horizontal from the center of rotation

    // for tracking position updates
    private double lastVerticalTicks;
    private double lastHorizontalTicks;
    private double currVerticalTicks;
    private double currHorizontalTicks;
    private double dVerticalTicks;
    private double dHorizontalTicks;

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
    }

    @Override
    public void reset() {
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        x = y = theta = 0;
    }

    @Override
    public void updatePosition() {

    }
}
