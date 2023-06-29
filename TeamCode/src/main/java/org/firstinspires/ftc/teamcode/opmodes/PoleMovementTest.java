package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumMovement;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Pole Movement Testing")
public class PoleMovementTest extends LinearOpMode {

    private CameraSubsystem camera;
    private MecanumMovement drive;

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        camera = new CameraSubsystem(hardwareMap);
        drive = new MecanumMovement(hardwareMap);

        // outputting values to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard
        boolean followPole = false;
        waitForStart();

        while (opModeIsActive()) {
            // user controls

            if (gamepad1.a){
                followPole = !followPole;
            }

            drive.move(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // reporting turret data
            telemetry.addData("followPole", followPole);
            telemetry.addLine();

            // reporting the pole detection and contour data
            telemetry.addLine("CAMERA DATA");
            if (camera.getPipeline().poleDetected() && followPole) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
                telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
                telemetry.addData("Contours found", tmp.size());
                telemetry.addLine("Largest Contour Data");
                telemetry.addData("Area", camera.getPipeline().largestContourArea());
                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
                telemetry.addData("Camera center", ContourPipeline.CENTER_X);
                telemetry.addLine();
                drive.adjustThetaCamera();

                // point camera towards the detected pole


            } else {
                if(camera.getPipeline().poleDetected()){
                    telemetry.addData("pole detected", "but not following");
                }
                else{
                    telemetry.addData("pole not detected", "not following");
                }
            }
            telemetry.update();
        }
    }
}
