package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.ContourPipeline;

public class MecanumMovement extends MecanumDriveSubsystem {

    private CameraSubsystem camera;
    public MecanumMovement(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
    }

    public void adjustToCoord(double x, double y, double theta) {
        double xPower = globalXPID.PIDOutput(x, odometry.getXPos());
        double yPower = globalYPID.PIDOutput(y, odometry.getYPos());
        double thetaPower = globalThetaPID.PIDOutput(theta, odometry.getTheta());
        fieldOrientedMove(xPower, yPower, thetaPower, odometry.getTheta());
    }

    public void adjustThetaCamera(){
        double thetaPower = cameraPID.PIDOutput(ContourPipeline.CENTER_X, (int)Math.round(camera.getPipeline().largestContourCenter().x));
        fieldOrientedMove(0, 0, thetaPower, odometry.getTheta());
    }


}
