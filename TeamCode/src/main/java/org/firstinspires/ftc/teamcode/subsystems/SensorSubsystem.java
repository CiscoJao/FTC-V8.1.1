package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem {

    private DistanceSensor dist;
    private BNO055IMU imu;

    public SensorSubsystem(HardwareMap hardwareMap) {
        // DISTANCE SENSOR
        dist = hardwareMap.get(DistanceSensor.class, "dist");

        // GYRO SENSOR
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getDist() {
        return dist.getDistance(DistanceUnit.CM);
    }

    public double getAngleDeg() {
        return imu.getAngularOrientation().firstAngle * (180/Math.PI);
    }

    public double getAngleRad() {
        return imu.getAngularOrientation().firstAngle;
    }
}
