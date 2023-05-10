package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="Contour detection testing")
public class Testing extends LinearOpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            turret.turn(gamepad1.right_stick_x);

            telemetry.addData("Right stick x", gamepad1.right_stick_x);
            telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
            telemetry.update();
        }
    }
}
