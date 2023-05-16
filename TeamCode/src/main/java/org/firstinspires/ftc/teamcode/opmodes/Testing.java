package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Francisco Testing")
public class Testing extends LinearOpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;
    private int targetPos;

    // todo delete later
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        targetPos = turret.getPosition();

        // for controlling buttons in continuous loops, maybe delete later
        boolean stateA, lastStateA = false;
        boolean stateB, lastStateB = false;

        // todo delete later
        timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            timer.reset(); // todo delete later
            sleep(20); // todo delet later

            // user controls
            if (gamepad1.right_trigger > 0) {
                turret.turn(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                turret.turn(-gamepad1.left_trigger);
            } else {
                //turret.stop();
                //turret.turnPID(turret.getPosition(), targetPos);
            }

            // manually raise or lower target position (for testing), maybe delete later
            stateA = gamepad1.a;
            if (stateA != lastStateA) {
                if (stateA)
                    targetPos += 90 * TurretSubsystem.DEG_TO_TICKS;
            }
            lastStateA = stateA;

            stateB = gamepad1.b;
            if (stateB != lastStateB) {
                if (stateB)
                    targetPos -= 90 * TurretSubsystem.DEG_TO_TICKS;
            }
            lastStateB = stateB;

            // reporting turret data
            telemetry.addData("Clock wise limit", turret.clockWiseLimit());
            telemetry.addData("Counter clock wise limit", turret.counterClockWiseLimit());
            telemetry.addData("Turret motor power", turret.getMotorPower());
            telemetry.addData("Turret position", turret.getPosition());
            telemetry.addData("Target position", targetPos);
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
                telemetry.addData("Camera center", ContourPipeline.CENTER_X);
                telemetry.addLine();

                // point camera towards the detected pole
                turret.followPID(ContourPipeline.CENTER_X, (int)Math.round(camera.getPipeline().largestContourCenter().x));

            } else {
                telemetry.addLine("No contours detected");
                turret.stop();
            }

            telemetry.addData("OpMode loop time", timer.milliseconds()); // todo delete later
            telemetry.update();
        }
    }
}
