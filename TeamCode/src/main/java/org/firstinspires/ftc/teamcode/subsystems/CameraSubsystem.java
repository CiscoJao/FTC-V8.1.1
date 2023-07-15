package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraSubsystem {
    public OpenCvCamera camera;
    private ContourPipeline contourPipeline; // pipeline designed to look for contours of the poles
    private int cameraMonitorViewId; // ID of the viewport which camera feed will be displayed

    public static final int VIEW_WIDTH = 320;
    public static final int VIEW_HEIGHT = 176;
    public static final int CENTER_X = VIEW_WIDTH / 2;
    public static final int CENTER_Y = VIEW_HEIGHT / 2;

    public CameraSubsystem(HardwareMap hardwareMap){

        contourPipeline = new ContourPipeline();

        // initiate the needed parameters
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initiate the camera object with created parameters and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // connect whichever pipeline is desired and comment the other one
        camera.setPipeline(contourPipeline);

        // runs camera on a separate thread so it can run simultaneously with everything else
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                // starts the camera stream when init is pressed
                camera.startStreaming(VIEW_WIDTH,VIEW_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public ContourPipeline getPipeline() {
        return contourPipeline;
    }
}
