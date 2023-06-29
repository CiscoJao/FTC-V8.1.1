package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ContourPipeline;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MecanumDriveSubsystem {

    protected ThreeWheelOdometry odometry;
    private static double kpx = 0.066;
    private static double kdx = 0.08;
    private static double kix = 0;
    private static double kpy = 0.08;
    private static double kdy = 0.0005;
    private static double kiy = 0;
    private static double kptheta = 2;
    private static double kdtheta = 0.2;
    private static double kitheta = 0.0;

    private static double kpCamera = 0.0001;
    private static double kdCamera = 0.0001;
    private static double kiCamera = 0.0001;

    protected static PIDController globalXPID = new PIDController(kpx, kdx, kix);
    protected static PIDController globalYPID = new PIDController(kpy, kdy, kiy);
    protected static PIDController globalThetaPID = new PIDController(kptheta, kdtheta, kitheta);
    protected static PIDController cameraPID = new PIDController(kpCamera, kdCamera, kiCamera);


    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta){
        MecanumDriveSubsystem.kpx = kpx;
        MecanumDriveSubsystem.kdx = kdx;
        MecanumDriveSubsystem.kix = kix;
        MecanumDriveSubsystem.kpy = kpy;
        MecanumDriveSubsystem.kdy = kdy;
        MecanumDriveSubsystem.kiy = kiy;
        MecanumDriveSubsystem.kptheta = kptheta;
        MecanumDriveSubsystem.kdtheta = kdtheta;
        MecanumDriveSubsystem.kitheta = kitheta;
        globalXPID.setConstant(kpx, kdx, kix);
        globalYPID.setConstant(kpy, kdy, kiy);
        globalThetaPID.setConstant(kptheta, kdtheta, kitheta);
        cameraPID.setConstant(kpCamera, kdCamera, kiCamera);
    }

    protected final DcMotor frontRight, frontLeft, backRight, backLeft;
    protected double frontRightPow, frontLeftPow, backRightPow, backLeftPow;
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
    public void fieldOrientedMove(double x, double y, double z, double theta) {
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
}

class MecanumMovement extends MecanumDriveSubsystem {

    private CameraSubsystem camera;
    public MecanumMovement(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
    }

    public void adjustToCoord(double x, double y, double theta) {
        double xPower = globalXPID.PIDOutput(x, odometry.getXPos());
        double yPower = globalYPID.PIDOutput(y, odometry.getYPos());
        double thetaPower = globalThetaPID.PIDOutput(theta, odometry.getTheta());
        fieldOrientedMove(xPower, yPower, thetaPower, odometry.getTheta());
    }

    public void adjustThetaCamera(){
        double thetaPower = cameraPID.PIDOutput(ContourPipeline.CENTER_X, (int)Math.round(camera.getPipeline().largestContourCenter().x));
        fieldOrientedMove(odometry.x, odometry.y, thetaPower, odometry.getTheta());
    }


}