package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name="Movement Testing")
public class MovementTest extends LinearOpMode {

    private MecanumDriveSubsystem drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDriveSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
