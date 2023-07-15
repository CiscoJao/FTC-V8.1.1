package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class MecanumDriveSubsystem {

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private double frontRightPow, frontLeftPow, backRightPow, backLeftPow;
    private final double SCALE = 0.7; // for scaling motor powers down

    private OdometrySubsystem odo;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController thetaPID;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, OdometrySubsystem odometry) {
        initialize(hardwareMap);
        odo = odometry;
    }

    // initialize motors and PID controllers
    private void initialize(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xPID = new PIDController(0.02, 0.00, 0.002);
        yPID = new PIDController(0, 0, 0);
        thetaPID = new PIDController(0, 0, 0);
    }

    private void turnOnInternalPID() {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnOffInternalPID() {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    // general move method that moves the robot relative to itself
    public void move(double x, double y, double z) {
        turnOnInternalPID();

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
    public void fieldOrientedMove(double x, double y, double z, double theta) {
        turnOnInternalPID();

        // translate the field relative movement (joystick) into the robot relative movement
        double newX = x * Math.cos(theta) + y * Math.sin(theta);
        double newY = - x * Math.sin(theta) + y * Math.cos(theta);

        frontRightPow = - newX + newY - z;
        frontLeftPow = newX + newY + z;
        backRightPow = newX + newY - z;
        backLeftPow = - newX + newY + z;

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

    public void moveToGlobalPosition(double targetX, double targetY, double targetTheta) {
        turnOffInternalPID();

        // field oriented drive axes are offset by 90 degrees (joystick x-axis controls robot y-axis)
        fieldOrientedMove(
                0,
                xPID.PIDOutput(odo.getXPos(), targetX),
                0,
                odo.getHeading()
        );
    }
}