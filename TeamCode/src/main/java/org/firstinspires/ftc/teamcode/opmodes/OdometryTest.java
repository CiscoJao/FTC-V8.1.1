package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
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
        odo.reset();
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
        //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        drive.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getAngleRAD());

        telemetry.addData("X centimeters", odo.getXPos());
        telemetry.addData("Y centimeters", odo.getYPos());
        telemetry.addData("Heading from Odo", Math.toDegrees(odo.getHeading()));
        telemetry.addData("Heading from IMU", imu.getAngleDEG());
        telemetry.addData("angular velocity", Math.toDegrees(imu.getAngularVelocity()));
        telemetry.update();
    }
}
