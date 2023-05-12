package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    private int currentTargetPos;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        currentTargetPos = turret.getPosition();

        waitForStart();

        while (opModeIsActive()) {

            // user controls
            if (gamepad1.right_trigger > 0) {
                turret.turn(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                turret.turn(-gamepad1.left_trigger);
            } else {
                //turret.stop();
                turret.turnPID(turret.getPosition(), currentTargetPos);

            }

            if (gamepad1.a) {
                //turret.turnByAngle(90);
                currentTargetPos += 90 * TurretSubsystem.DEG_TO_TICKS;
            } else if (gamepad1.b) {
                //turret.turnByAngle(-90);
                currentTargetPos -= 90 * TurretSubsystem.DEG_TO_TICKS;
            }

            // reporting turret data
            telemetry.addData("Clock wise limit", turret.clockWiseLimit());
            telemetry.addData("Counter clock wise limit", turret.counterClockWiseLimit());
            telemetry.addData("Turret motor power", turret.getMotorPower());
            telemetry.addData("Turret position", turret.getPosition());
            telemetry.addData("Target position", currentTargetPos);
            telemetry.addLine();

            // reporting the pole detection and contour data
            if (camera.getPipeline().poleDetected()) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
                telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
                telemetry.addData("Contours found", tmp.size());
                telemetry.addLine("Largest Contour Data");
                telemetry.addData("Area", camera.getPipeline().largestContourArea());
                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
                telemetry.addLine();
            } else {
                telemetry.addLine("No contours detected");
            }

            telemetry.update();
        }
    }
}
