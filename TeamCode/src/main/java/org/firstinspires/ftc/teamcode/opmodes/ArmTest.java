package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmTest extends LinearOpMode {

    DcMotor arm;
    Servo right;
    Servo left;

    @Override
    public void runOpMode() throws InterruptedException {

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.FORWARD);
        right.scaleRange(0, 1);
        left.scaleRange(0, 1);

        waitForStart();

        right.setPosition(0.5);
        left.setPosition(0.5);

        while(opModeIsActive()) {
            arm.setPower(gamepad1.right_stick_y);

            if (gamepad1.a) {
                right.setPosition(0);
                left.setPosition(0);
            } else if (gamepad1.b) {
                right.setPosition(1);
                left.setPosition(1);
            }
        }

    }
}
