package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Pole Detection Test")
public class PoleDetectionTest extends LinearOpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;
    private MecanumDriveSubsystem drive;
    private SensorSubsystem sensor;
    private IMUSubsystem imu;
    private LEDSubsystem led;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
        sensor = new SensorSubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);

        // outputting values and camera stream to FTCDashboard for debugging
        FtcDashboard.getInstance().startCameraStream(camera.camera, 30);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        waitForStart();

        while (opModeIsActive()) {
            // user controls
            if (gamepad1.right_trigger > 0) {
                turret.turn(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                turret.turn(-gamepad1.left_trigger);
            }

            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeadingRAD());

            // reporting turret data
            telemetry.addLine("TURRET DATA");
            telemetry.addData("Turret motor power", turret.getMotorPower());
            telemetry.addData("Turret position", turret.getPosition());
            telemetry.addData("Distance sensor", sensor.getDistanceCM());
            telemetry.addLine();

            // reporting the pole detection and contour data
            telemetry.addLine("CAMERA DATA");
            if (camera.getPipeline().poleDetected()) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
                telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
                telemetry.addData("Contours found", tmp.size());
                telemetry.addLine("Largest Contour Data");
                telemetry.addData("Area", camera.getPipeline().largestContourArea());
                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
                telemetry.addData("Camera center", CameraSubsystem.CENTER_X);
                telemetry.addLine();

                // point camera towards the detected pole
                turret.followPID(CameraSubsystem.CENTER_X, (int)Math.round(camera.getPipeline().largestContourCenter().x));
            } else {
                telemetry.addLine("No contours detected");
                turret.stop();
            }

            // LED indicator if the turret is centered on a pole (with tolerance of 20 pixels)
            if (Math.abs(CameraSubsystem.CENTER_X - (int) Math.round(camera.getPipeline().largestContourCenter().x)) < 20
                    && camera.getPipeline().poleDetected()) {
                led.greenLight();
                telemetry.addLine("Green light!");
            } else {
                led.redLight();
                telemetry.addLine("Red light!");
            }

            telemetry.update();
        }
    }
}