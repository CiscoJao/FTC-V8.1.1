package org.firstinspires.ftc.teamcode.subsystems;

/*
Parent abstract odometry class that lays the foundation for both 2 wheel and 3 wheel configurations for FTC.
An abstract class is a special class that CANNOT create objects, it MUST be inherited by another child class
in order to use its methods and attributes.
Abstract classes can be used to organize code and keep it neat. In this case, this abstract class provides
the common methods and attributes that both 2 wheel and 3 wheel odometry classes will use.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public abstract class Odometry {
    public DcMotor lEncoder, rEncoder, bEncoder;

    public static final double WHEEL_DIAMETER = 3.5; // in centimeters
    public static final double TICKS_PER_REV = 8192;
    public static final double GEAR_RATIO = 1;
    public static final double TICKS_TO_CM = (Math.PI * WHEEL_DIAMETER * GEAR_RATIO) / TICKS_PER_REV; // circumference over ticks

    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public double tempX = 0;
    public double tempY = 0;
    public double dx = 0;
    public double dy = 0;
    public double dTheta = 0;
    public int lEncoderi = 0;
    public int rEncoderi = 0;
    public int bEncoderi = 0;
    public int lEncoderf = 0;
    public int rEncoderf = 0;
    public int bEncoderf = 0;

    public ElapsedTime time;

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
    double X_TUNER = 1;
    double Y_TUNER = 1;

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

class OdometrySubSystem extends Odometry {

    public void reset() {
        // reset encoders
        x = y = theta = 0;
    }

    public void updatePosition() {
        // update position
        time.reset();
        lEncoderf = lEncoder.getCurrentPosition();
        rEncoderf = rEncoder.getCurrentPosition();
        bEncoderf = bEncoder.getCurrentPosition();
        dx = dxc*((lEncoderf-lEncoderi)+(rEncoderf-rEncoderi));
        dTheta = dThetac*((rEncoderf-rEncoderi)-(lEncoderf-lEncoderi)); //unit circle direction
        dy = (dyc*(bEncoderf-bEncoderi))+(lengthFromOdometrySideToFront*dTheta);
        tempX = (dx*Math.cos(theta))+(dy*Math.sin(theta));
        tempY = (dy*Math.cos(theta))-(dx*Math.sin(theta));
        x = x+tempX;
        y = y+tempY;
        theta += dTheta;
        if (theta > 2*Math.PI){
            theta -= 2*Math.PI;
        } else if (theta < 0){
            theta += 2*Math.PI;
        }
        rEncoderi = rEncoderf;
        lEncoderi = lEncoderf;
        bEncoderi = bEncoderf;
    }

}
