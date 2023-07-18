package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Disabled
@Autonomous(name="PID move test")
public class PIDMovementTest extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu;
    private OdometrySubsystem odo;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        odo = new OdometrySubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, odo);

        drive.turnOffInternalPID();
        imu.resetHeading();
        odo.reset();

        // allows telemetry to output to phone and dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        // multi threads in LinearOpmode
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            Executor executor = Executors.newFixedThreadPool(4);
            CompletableFuture.runAsync(this::updateOdometry, executor);
            CompletableFuture.runAsync(this::updateTelemetry, executor);
        }

        drive.moveToGlobalPosition(70, 0, 0);
        sleep(4000);
        drive.moveToGlobalPosition(70, -50, 0);
        sleep(4000);
        drive.moveToGlobalPosition(70, -50, -Math.PI/2);
        sleep(4000);
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
            telemetry.update();
        }
    }
}
