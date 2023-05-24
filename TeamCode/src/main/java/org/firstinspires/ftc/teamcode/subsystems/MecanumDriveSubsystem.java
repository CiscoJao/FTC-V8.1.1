package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem {

    private final DcMotor frontRight, frontLeft, backRight, backLeft;
    private double frontRightPow, frontLeftPow, backRightPow, backLeftPow;
    private final double SCALE = 0.75; // for scaling motor powers down

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // general move method that moves the robot relative to itself
    public void move(double x, double y, double z) {
        // find needed motor powers with joystick vectors
        frontRightPow = - x + y - z;
        frontLeftPow = x + y + z;
        backRightPow = x + y - z;
        backLeftPow = - x + y + z;

        // scale motor powers down to keep in range of -1 < power < 1
        double largest = Math.max(
                Math.max(Math.abs(frontRightPow), Math.abs(frontLeftPow)),
                Math.max(Math.abs(backRightPow), Math.abs(backLeftPow)));
        if (largest > 1) {
            frontRightPow /= largest;
            frontLeftPow /= largest;
            backRightPow /= largest;
            backLeftPow /= largest;
        }

        frontRight.setPower(frontRightPow * SCALE);
        frontLeft.setPower(frontLeftPow * SCALE);
        backRight.setPower(backRightPow * SCALE);
        backLeft.setPower(backLeftPow * SCALE);
    }

    // general move method that moves the robot relative to the field
    public void fieldOrientedMove() {

    }
}