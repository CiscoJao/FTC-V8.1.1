package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem {

    private final DcMotor frontRight, frontLeft, backRight, backLeft;
    private double frontRightPow, frontLeftPow, backRightPow, backLeftPow;
    private final double SCALE = 0.5; // for scaling motor powers down
    private final double CLIP_MAX = 1.0;
    private final double CLIP_MIN = -1.0;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotor.class, "");
        frontLeft = hardwareMap.get(DcMotor.class, "");
        backRight = hardwareMap.get(DcMotor.class, "");
        backLeft = hardwareMap.get(DcMotor.class, "");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
