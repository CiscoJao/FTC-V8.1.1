package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@Autonomous(name="PID move test")
public class PIDMovementTest extends ThreadOpMode {

    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu;
    private OdometrySubsystem odo;

    @Override
    public void mainInit() {
        imu = new IMUSubsystem(hardwareMap);
        odo = new OdometrySubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap, odo);

        imu.resetHeading();
        odo.reset();

        registerThread(new TaskThread(() -> odo.updatePosition()));
    }

    @Override
    public void mainLoop() {
        drive.moveToGlobalPosition(50, 0, 0);

        telemetry.addData("X position (CM)", odo.getXPos());
        telemetry.update();
    }
}
