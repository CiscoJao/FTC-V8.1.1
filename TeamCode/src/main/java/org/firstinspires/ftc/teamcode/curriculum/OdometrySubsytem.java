package org.firstinspires.ftc.teamcode.curriculum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometrySubsytem {

    // constants that define the geometry of the robot
    public static final double WHEEL_DIAMETER = 3.5; // in centimeters
    public static final double TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 1;
    public static final double TICKS_TO_CM = (Math.PI * WHEEL_DIAMETER * GEAR_RATIO) / TICKS_PER_REV;

    /*
    One way to accurately find these lengths is to pivot the robot (this will only work on symmetrical
    robots) in a perfect 360 circle. You can either do this by hand, or with the IMU.
    Output the raw tick counts of each encoder to telemetry and record them. This is the circumference
    of a circle in encoder ticks. Then divide the values by 2π and multiply by TICKS_TO_CM. This will give
    you the "radius" of the circle the encoder travelled in CM. The "radius" is the same as your L1/L2/B.
     */
    private final double L1 = 1; // distance of right encoder from the center of rotation in CM
    private final double L2 = 1; // distance of left encoder from the center of rotation in CM
    private final double B = 1; // distance of horizontal encoder from the center of rotation in CM

    // for tracking the position of the robot
    private double x, y, theta;
    private double dx, dy, dtheta;

    // for tracking the position updates from the encoders
    private double lastRightTicks, lastLeftTicks, lastAuxTicks;
    private double currRightTicks, currLeftTicks, currAuxTicks;
    private double dRightTicks, dLeftTicks, dAuxTicks;

    // encoder objects
    public final DcMotor right;
    public final DcMotor left;
    public final DcMotor aux;

    public OdometrySubsytem(HardwareMap hardwareMap) {
        right = hardwareMap.get(DcMotor.class, "rightOdo");
        left = hardwareMap.get(DcMotor.class, "leftOdo");
        aux = hardwareMap.get(DcMotor.class, "horizontalOdo");

        // directions are subject to change depending on how you assemble the robot
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        aux.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
    You should call reset() upon the initialization of your Autonomous. However, you should NOT call
    this during the initialization of your TeleOp. This way, you can save the position of your robot
    when transitioning between Autonomous and TeleOp.

    Alternatively, to solve encoder drift issues, you can have the driver manually call this method
    during TeleOp using a button on the game-pad. If the heading starts to drift significantly, the
    driver should reset the heading.
     */
    public void reset() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        x = y = theta = 0;
    }

    // this method is designed to be called in a constant loop
    public void update() {
        // step 1: read tick counts from encoders
        currRightTicks = right.getCurrentPosition();
        currLeftTicks = left.getCurrentPosition();
        currAuxTicks = aux.getCurrentPosition();

        // step 2: find the change in ticks since the last update
        dRightTicks = currRightTicks - lastRightTicks;
        dLeftTicks = currLeftTicks - lastLeftTicks;
        dAuxTicks = currAuxTicks - lastAuxTicks;

        // step 3: use our ∆ticks in our derived movement equations
        dtheta = TICKS_TO_CM * (dRightTicks - dLeftTicks) / (L1 + L2);
        dx = TICKS_TO_CM * (dRightTicks + dLeftTicks) / 2;
        dy = TICKS_TO_CM * dAuxTicks + B * dtheta;

        // step 4: rotate relative coordinates to field coordinates, while also updating the position
        x += dx * Math.cos(theta) - dy * Math.sin(theta);
        y += dx * Math.sin(theta) + dy * Math.cos(theta);
        theta += dtheta;

        // record the previous encoder positions for the next update
        lastRightTicks = currRightTicks;
        lastLeftTicks = currLeftTicks;
        lastAuxTicks = currAuxTicks;
    }

    public double getXPos() {
        return x;
    }

    public double getYPos() {
        return y;
    }

    public double getHeading() {
        return theta;
    }
}
