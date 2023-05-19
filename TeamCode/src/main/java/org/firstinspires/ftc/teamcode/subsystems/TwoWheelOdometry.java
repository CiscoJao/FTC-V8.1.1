package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelOdometry extends Odometry{

    private final DcMotor vertical;
    private final DcMotor horizontal;
    private final IMUSubsystem imu;

    public TwoWheelOdometry(HardwareMap hardwareMap) {
        imu = new IMUSubsystem(hardwareMap);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        horizontal = hardwareMap.get(DcMotor.class, "horizontal");

        vertical.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontal.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        In some cases, you don't want to reset the encoders on initialization, this is because
        the robot will initialize the odometry system between different OpModes.
        If the robot were to move during autonomous period, it would be ideal to save its position on
        the field when transitioning from Auto to TeleOp.
         */
        reset();

        /*
        NOTE: x, y, and theta have already been initialized to zero in the constructor of the parent
        class, which will automatically be called when the constructor of this class is called.
        */
    }

    @Override
    public void reset() {
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theta = 0; // todo this is not the actual way to reset this, change later
    }

    @Override
    public void updatePosition() {
        // todo these only track motion relative to robot, NOT field relative
        x = vertical.getCurrentPosition() * TICKS_TO_CM;
        y = horizontal.getCurrentPosition() * TICKS_TO_CM;
        theta = imu.getAngleDEG();
    }
}
