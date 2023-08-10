package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name="Limit Switch Test")
public class MagneticSwitchTest extends LinearOpMode {

    private DigitalChannel limit1;
    private TurretSubsystem turret;

    private int counter = 0;

    private boolean limitState;
    private boolean lastLimitState;
    private int direction;
    private final double SCALE = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        limit1 = hardwareMap.get(DigitalChannel.class, "limit1");
        limit1.setMode(DigitalChannel.Mode.INPUT);

        turret = new TurretSubsystem(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            direction = counter % 2 == 0 ? 1 : -1;

            turret.turn(direction * SCALE);

            limitState = limit1.getState();
            if (limitState != lastLimitState) {
                if (!limitState) {
                    counter++;
                }
            }
            lastLimitState = limitState;

            telemetry.addData("Limit 1", limit1.getState());
            telemetry.addData("Counter", counter);
            telemetry.addData("Turret Position", turret.getPosition());
            telemetry.update();
        }

    }
}
