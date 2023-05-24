package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odo;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@TeleOp(name="Odo Test")
public class OdometryTest extends ThreadOpMode {

    private TwoWheelOdometry odo;
    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu; // todo delete later

    @Override
    public void mainInit() {

        odo = new TwoWheelOdometry(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap); // todo delete later

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
        drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("X centimeters", odo.getXPos());
        telemetry.addData("Y centimeters", odo.getYPos());
        telemetry.addData("heading in degrees", odo.getHeading() * 180 / Math.PI);
        telemetry.addData("angular velocity", imu.getAngularVelocity());
        telemetry.update();
    }
}
