package org.firstinspires.ftc.teamcode.subsystems;

/*
Parent abstract odometry class that lays the foundation for both 2 wheel and 3 wheel configurations for FTC.
An abstract class is a special class that CANNOT create objects, it MUST be inherited by another child class
in order to use its methods and attributes.
Abstract classes can be used to organize code and keep it neat. In this case, this abstract class provides
the common methods and attributes that both 2 wheel and 3 wheel odometry classes will use.
 */

public abstract class Odometry {

    public static final double WHEEL_DIAMETER = 3.5; // in centimeters
    public static final double TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 1;
    public static final double TICKS_TO_CM = (Math.PI * WHEEL_DIAMETER * GEAR_RATIO) / TICKS_PER_REV; // circumference over ticks
    double x, y, theta;
    double dx, dy, dtheta;

    /*
    These tuning values are used to further calibrate the odometry pods. There will always be external
    factors such as size tolerances, friction, and assembly errors. These tuning values can found
    by rolling the robot by hand beside a set distance with a measuring tape and comparing the
    theoretical distance with the measured distance as a ratio.
    THEORETICAL DISTANCE / MEASURED DISTANCE
    These tuning values can be further improved on by running multiple tests and measurements and
    getting an average of those results. But bare in mind that this will never 100% correct all
    accuracy errors, only minimize them.
     */
    double X_TUNER;
    double Y_TUNER;

    public Odometry() {
        x = y = theta = 0;
    }

    /*
    In some cases, you don't want to reset the encoders on initialization, this is because
    the robot will initialize the odometry system between different OpModes.
    If the robot were to move during autonomous period, it would be ideal to save its position on
    the field when transitioning from Auto to TeleOp. The encoders actually store the last recorded
    tick count between initializations!
     */
    public abstract void reset();
    public abstract void updatePosition();

    public double getXPos() {
        return x * X_TUNER;
    }

    public double getYPos() {
        return y * Y_TUNER;
    }

    public double getHeading() {
        return theta;
    }
}
