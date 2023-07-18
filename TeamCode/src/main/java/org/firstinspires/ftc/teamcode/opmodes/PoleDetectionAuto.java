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

        drive.moveToGlobalPosition(90, 0, 0);
        sleep(2000);
        drive.turnToPole(camera);
        sleep(2000);

        //while (opModeIsActive() )
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
            telemetry.addData("DIstance", sensor.getDistanceCM());
            telemetry.update();
        }
    }
}
