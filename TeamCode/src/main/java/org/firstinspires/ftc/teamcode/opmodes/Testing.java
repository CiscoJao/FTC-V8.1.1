package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Francisco Testing")
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
            telemetry.addData("Touch Sensor", turret.isTouched());

            // reporting the contour areas
            if (camera.getPipeline().poleDetected()) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
                telemetry.addLine();
                telemetry.addData("Contours found", tmp.size());
                telemetry.addLine("Largest Contour Data");
                telemetry.addData("Area", camera.getPipeline().largestContourArea());
                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
                telemetry.addLine();

                // todo delete later
                for (int i = 0; i < tmp.size(); i++) {
                    telemetry.addData("Contour Area", tmp.get(i));
                }
            } else {
                telemetry.addLine("No contours detected");
            }

            telemetry.update();
        }
    }
}
