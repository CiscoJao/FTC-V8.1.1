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
    public static final double TICKS_TO_CM = (Math.PI * WHEEL_DIAMETER * GEAR_RATIO) / TICKS_PER_REV; // circumference / ticks
    double x, y, theta;
    double dx, dy, dtheta;

    /*
        These tuning values are used to further calibrate the odometry pods. There will always be external
        factors such as size tolerances, friction, and assembly errors. These tuning values were found
        by rolling the robot by hand beside a set distance with a measuring tape and comparing the
        theoretical distance with the measured distance.
        THEORETICAL DISTANCE / MEASURED DISTANCE
         */
    double X_TUNER = 1;
    double Y_TUNER = 1;

    public Odometry() {
        x = y = theta = 0;
    }

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
