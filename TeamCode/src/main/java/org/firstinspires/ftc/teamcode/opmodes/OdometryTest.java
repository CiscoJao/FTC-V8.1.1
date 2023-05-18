package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

public class OdometryTest extends ThreadOpMode {

    private OdometrySubsystem odo;

    @Override
    public void mainInit() {

        odo = new OdometrySubsystem(hardwareMap);

        // starting odometry on a separate thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                odo.updatePosition();
            }
        }));
    }

    @Override
    public void mainLoop() {
        telemetry.addData("centimeters", odo.getX());
    }
}
