package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@TeleOp(name="Odo Test")
public class OdometryTest extends ThreadOpMode {

    private MecanumDriveSubsystem drive;
    private OdometrySubsystem odo;
    private IMUSubsystem imu; // for testing the heading with another source

    @Override
    public void mainInit() {

        drive = new MecanumDriveSubsystem(hardwareMap);
        odo = new OdometrySubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);

        drive.turnOnInternalPID();
        odo.reset();
        imu.resetHeading();

        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        // continuously update odometry on a separate thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                odo.updatePosition();
            }
        }));
    }

    @Override
    public void mainLoop() {
        //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeadingRAD());

        telemetry.addLine("Odometry Data");
        telemetry.addData("X position", odo.getXPos());
        telemetry.addData("Y position", odo.getYPos());
        telemetry.addData("Heading", odo.getHeading() / Math.PI);
        telemetry.addData("Right ticks", odo.right.getCurrentPosition());
        telemetry.addData("Left ticks", odo.left.getCurrentPosition());
        telemetry.addData("Aux ticks", odo.aux.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("IMU Data");
        telemetry.addData("Heading DEG", imu.getHeadingDEG());
        telemetry.addData("Heading RAD", imu.getHeadingRAD() / Math.PI);
        telemetry.addLine();
        telemetry.addLine("Motor Data");
        telemetry.addData("front left", drive.getFrontLeftPow());
        telemetry.addData("front right", drive.getFrontRightPow());
        telemetry.addData("back left", drive.getBackLeftPow());
        telemetry.addData("back right", drive.getBackRightPow());
        telemetry.update();
    }
}
