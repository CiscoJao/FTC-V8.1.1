package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Config
@Autonomous
public class ThreePolesTest extends LinearOpMode {

    private CameraSubsystem camera;
    private MecanumDriveSubsystem drive;

    private ThreeWheelOdometry odometry;

    public static double kp = 0.005;
    public static double kd = 0.0;
    public static double ki = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new ThreeWheelOdometry(hardwareMap);
        odometry.reset();
        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.camera, 24);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard
        drive.setCamera(kp, kd, ki);
        waitForStart();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            Executor executor = Executors.newFixedThreadPool(4);
            CompletableFuture.runAsync(this::runOdometry, executor);
        }
    }

    public void runOdometry(){
        while(opModeIsActive()) {
            odometry.updatePosition();
            telemetry.addData("x", odometry.getXPos());
            telemetry.addData("y", odometry.getYPos());
            telemetry.addData("theta", odometry.getTheta());
//            telemetry.addData("following", following);
//            telemetry.addData("kp", kp);
//            telemetry.addData("kd", kd);
//            telemetry.addData("ki", ki);
//            telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
//            telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
//            telemetry.addData("Camera center", ContourPipeline.CENTER_X);
//            telemetry.addData("camera error", camera.getPipeline().largestContourCenter().x - ContourPipeline.CENTER_X);
//            telemetry.addData("thetapower",drive.thetapower);
//            telemetry.addData("testError", drive.testError);
            telemetry.update();
        }
    }
}
