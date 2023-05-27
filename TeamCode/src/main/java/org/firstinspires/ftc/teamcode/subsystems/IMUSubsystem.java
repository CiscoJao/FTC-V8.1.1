package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUSubsystem {

    private final BNO055IMU imu;

    public IMUSubsystem(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getAngleRAD() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getAngleDEG() {
        return imu.getAngularOrientation().firstAngle * 180 / Math.PI;
    }

    // units are radians per second
    public double getAngularVelocity() {
        return imu.getAngularVelocity().zRotationRate;
    }
}
