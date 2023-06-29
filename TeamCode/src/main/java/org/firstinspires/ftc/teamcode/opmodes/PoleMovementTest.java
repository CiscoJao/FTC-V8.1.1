package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Config
@TeleOp(name="Pole Movement Testing")
public class PoleMovementTest extends LinearOpMode {

    private CameraSubsystem camera;
    private MecanumDriveSubsystem drive;

    private ThreeWheelOdometry odometry;

    public static double kp;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);

        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        waitForStart();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            CompletableFuture.runAsync(this::runOdometry);
        }

        boolean following = false;

        while (opModeIsActive()) {
            telemetry.addData("following", following);
            // user controls
            if(gamepad1.a){
                following = !following;
            }

            drive.move(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // reporting the pole detection and contour data
            telemetry.addLine("CAMERA DATA");
            if (camera.getPipeline().poleDetected() && following) {
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
                drive.adjustThetaCamera(camera);

            } else {
                telemetry.addLine("No contours detected");
            }

            telemetry.update();
        }
    }

    public void runOdometry(){
        odometry.updatePosition();
    }
}
