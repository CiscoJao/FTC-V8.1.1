package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreeWheelOdometry extends Odometry {

    private final DcMotor right;
    private final DcMotor left;
    private final DcMotor aux;

    public double getLeft(){
        return left.getCurrentPosition();
    }

    public double getRight(){
        return right.getCurrentPosition();
    }

    public double getAux(){
        return aux.getCurrentPosition();
    }

    public double getDx(){
        return dx;
    }

    public double getDy(){
        return dy;
    }

    public double getTheta(){
        return theta;
    }

    public double getDTheta(){
        return dtheta;
    }

    // constants that define geometry of the robot
    private final double L1 = 1; // distance of right encoder from the center of rotation
    private final double L2 = 20.15618755; // distance of left encoder from the center of rotation
    private final double L3 = 17.87437649; // distance of horizontal encoder from the center of rotation

    // for tracking position updates
    private double lastRightTicks;
    private double lastLeftTicks;
    private double lastAuxTicks;
    private double currRightTicks;
    private double currLeftTicks;
    private double currAuxTicks;
    private double dRightTicks;
    private double dLeftTicks;
    private double dAuxTicks;

    public ThreeWheelOdometry(HardwareMap hardwareMap) {
        right = hardwareMap.get(DcMotor.class, "rightOdo");
        left = hardwareMap.get(DcMotor.class, "leftOdo");
        aux = hardwareMap.get(DcMotor.class, "horizontalOdo");

        // directions are subject to change depending on how you assemble the robot
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        aux.setDirection(DcMotorSimple.Direction.FORWARD);

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
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         x = y = theta = 0;
    }

    @Override
    public void updatePosition() {

        // finding the total distance (S) encoders have travelled in ticks since the last update
        currRightTicks = right.getCurrentPosition();
        currLeftTicks = left.getCurrentPosition();
        currAuxTicks = aux.getCurrentPosition();

        dRightTicks = currRightTicks - lastRightTicks;
        dLeftTicks = currLeftTicks - lastLeftTicks;
        dAuxTicks = currAuxTicks - lastAuxTicks;

        // finding the changes in position since the last update using the derived movement equations
        dtheta = TICKS_TO_CM * (dRightTicks - dLeftTicks) / (L2*2);
        dx = TICKS_TO_CM * (dRightTicks + dLeftTicks) / 2;
        dy = TICKS_TO_CM * dAuxTicks + L3 * dtheta;

        // add the small change in position to the overall position, while also accounting for field orientation
        // NOTE: remember Java Math uses RADIANS, ensure that all theta values are in RAD
        x += dx * Math.cos(theta) - dy * Math.sin(theta);
        y += dx * Math.sin(theta) + dy * Math.cos(theta);
        theta += dtheta;

        // todo include, clip theta in the range of -180 < theta < 180 if over
        if (theta > Math.PI*2){
            theta -= Math.PI*2;
        } else if (theta < 0){
            theta += Math.PI*2;
        }

        // record the previous encoder positions for the next update
        lastRightTicks = currRightTicks;
        lastLeftTicks = currLeftTicks;
        lastAuxTicks = currAuxTicks;
    }
}

class GyroOdometry extends ThreeWheelOdometry {

    public GyroOdometry(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void updatePosition() {
        
    }
}
