package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Disabled
@TeleOp(name="Movement Testing")
public class MovementTest extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDriveSubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);

        imu.resetHeading();

        waitForStart();

        while (opModeIsActive()) {
            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeadingRAD());

            telemetry.addData("Heading in DEG", imu.getHeadingDEG());
            telemetry.update();
        }
    }
}
