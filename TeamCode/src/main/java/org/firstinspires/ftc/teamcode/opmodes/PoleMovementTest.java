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

    public static double kp = 0.005;
    public static double kd = 0.0;
    public static double ki = 0;

    public static double kpx = 0.001;
    public static double kdx = 0.001;
    public static double kix = 0;
    public static double kpy = 0.001;
    public static double kdy = 0.0001;
    public static double kiy = 0;
    public static double kptheta = 0.5;
    public static double kdtheta = 0.002;
    public static double kitheta = 0.0;

    boolean following = false;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        odometry = new ThreeWheelOdometry(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, odometry);
        drive.setConstants(kpx, kdx, kix, kpy, kdy, kiy, kptheta, kdtheta, kitheta);
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

        while (opModeIsActive()) {
            // user controls
            if(gamepad1.right_bumper){
                following = !following;
            }

            drive.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, odometry.getHeading());

            // reporting the pole detection and contour data
//            telemetry.addLine("CAMERA DATA");
//            drive.adjustToCoord(60, 0, 0);
            if (camera.getPipeline().poleDetected() && following) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
////                telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
////                telemetry.addData("Contours found", tmp.size());
////                telemetry.addLine("Largest Contour Data");
////                telemetry.addData("Area", camera.getPipeline().largestContourArea());
////                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
////                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
////                telemetry.addData("Camera center", ContourPipeline.CENTER_X);
////                telemetry.addLine();
//
//                // point camera towards the detected pole
                if(gamepad1.right_bumper){
                    following = !following;
                }
                drive.adjustThetaCamera(camera, following);
//
            }/* else {
                telemetry.addLine("No contours detected");
            }*/
        }
    }

    public void runOdometry(){
        while(opModeIsActive()) {
            odometry.updatePosition();
            telemetry.addData("xcurrent", drive.xcurrent);
            telemetry.addData("ycurrent", drive.ycurrent);
            telemetry.addData("thetacurrent", drive.thetacurrent);
            telemetry.addData("x", odometry.getXPos());
            telemetry.addData("y", odometry.getYPos());
            telemetry.addData("theta", odometry.getHeading());
            telemetry.addData("x power", drive.thePowers[0]);
            telemetry.addData("y power", drive.thePowers[1]);
            telemetry.addData("theta power", drive.thePowers[2]);
//            telemetry.addData("x", odometry.getXPos());
//            telemetry.addData("y", odometry.getYPos());
//            telemetry.addData("theta", odometry.getHeading());
//            telemetry.addData("imutheta", odometry.getHeading());
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
