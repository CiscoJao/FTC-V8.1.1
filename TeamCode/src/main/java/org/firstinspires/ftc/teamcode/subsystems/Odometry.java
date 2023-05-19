package org.firstinspires.ftc.teamcode.subsystems;

/*
Parent abstract odometry class that lays the foundation for both 2 wheel and 3 wheel configurations for FTC.
An abstract class is a special class that CANNOT create objects, it MUST be inherited by another child class
in order to use its methods and attributes.
Abstract classes can be used to organize code and keep it neat. In this case, this abstract class provides
the common methods and attritubes that both 2 wheel and 3 wheel odometry classes can share.
 */

public abstract class Odometry {

    // circumference (2πr) over ticks
    public static final double TICKS_TO_CM = 2 * Math.PI * (3.5 / 2) / 8192;
    double x;
    double y;
    double theta;

    public Odometry() {
        x = y = theta = 0;
    }

    public abstract void reset();
    public abstract void updatePosition();

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
