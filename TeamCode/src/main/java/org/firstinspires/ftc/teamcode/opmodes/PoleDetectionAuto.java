package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/*
This OpMode consist of moving the robot autonomously and having it choose the largest yellow object
in its field of view. It will turn to the largest one, then approach it slowly until it senses
it is close enough.

Uses the camera, contour pipeline, object detection algorithm. Uses odometry. Uses PID for movement.
 */

@Autonomous(name="Pole Detection Auto Test")
public class PoleDetectionAuto extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu;
    private OdometrySubsystem odo;
    private CameraSubsystem camera;
    private SensorSubsystem sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        odo = new OdometrySubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, odo);
        camera = new CameraSubsystem(hardwareMap);
        sensor = new SensorSubsystem(hardwareMap);

        drive.turnOffInternalPID();
        imu.resetHeading();
        odo.reset();

        // allows telemetry to output to phone and dashboard
        FtcDashboard.getInstance().startCameraStream(camera.camera, 30);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        // multi threads in LinearOpmode
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            Executor executor = Executors.newFixedThreadPool(4);
            CompletableFuture.runAsync(this::updateOdometry, executor);
            CompletableFuture.runAsync(this::updateTelemetry, executor);
        }

        sleep(1000);
        drive.moveTowardsLargestObject(camera, sensor);
        sleep(1000);
/*
        // move forward towards the objects
        drive.moveToGlobalPosition(75, 0, 0);
        sleep(2000);

        // turn to the largest object detected
        drive.turnToLargestObject(camera);
        sleep(2000);

        // start to approach it slowly
        drive.move(0, 0.3, 0);

        // interrupt the code until the robot is close enough to the object
        while (opModeIsActive()) {
            if (sensor.getDistanceCM() < 20)
                break;
        }
        drive.stop();
        sleep(1000);
 */

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            odo.updatePosition();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("X position (CM)", odo.getXPos());
            telemetry.addData("Y position (CM)", odo.getYPos());
            telemetry.addData("Heading (RAD)", odo.getHeading());
            telemetry.addData("Pipeline time", camera.getPipeline().getProcessTime());
            telemetry.addData("Distance", sensor.getDistanceCM());
            telemetry.update();
        }
    }
}
