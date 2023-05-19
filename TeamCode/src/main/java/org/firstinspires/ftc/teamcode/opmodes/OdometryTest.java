package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Odo;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@TeleOp(name="Odo Test")
public class OdometryTest extends ThreadOpMode {

    private Odo odo;

    @Override
    public void mainInit() {

        odo = new Odo(hardwareMap);

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
        telemetry.addData("X centimeters", odo.getX());
        telemetry.addData("Y centimeters", odo.getY());
        telemetry.addData("heading in degrees", odo.getHeading());
    }
}
