package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometrySubsystem {

    private final DcMotor vertical;
    private final DcMotor horizontal;
    private final IMUSubsystem imu;

    private final double TICKS_TO_CM = 0;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        vertical = hardwareMap.get(DcMotor.class, "vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        imu = new IMUSubsystem(hardwareMap);
    }

}
