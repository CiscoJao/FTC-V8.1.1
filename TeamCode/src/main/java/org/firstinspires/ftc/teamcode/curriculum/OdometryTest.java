package org.firstinspires.ftc.teamcode.curriculum;

import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

public class OdometryTest extends ThreadOpMode {

    OdometrySubsytem odo;

    @Override
    public void mainInit() {

        odo = new OdometrySubsytem(hardwareMap);

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                odo.update();
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                // run another subsystem here or run another set of controls from the gamepad
            }
        }));
    }

    @Override
    public void mainLoop() {
        telemetry.addData("X position", odo.getXPos());
        telemetry.addData("Y position", odo.getYPos());
        telemetry.addData("Heading in DEG", odo.getHeading() * 180 / Math.PI);
        telemetry.update();
    }
}
