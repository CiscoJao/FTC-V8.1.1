package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;
import org.firstinspires.ftc.teamcode.util.YellowPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraSubsystem {
    public OpenCvCamera camera;
    private ContourPipeline contourPipeline; // pipeline designed to look for contours of the poles
    private YellowPipeline yellowPipeline; // pipeline designed to filter out the yellow pole
    int cameraMonitorViewId; // ID of the viewport which camera feed will be displayed

    public CameraSubsystem(HardwareMap hardwareMap){

        contourPipeline = new ContourPipeline();
        yellowPipeline = new YellowPipeline();

        // initiate the needed parameters
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initiate the camera object with created parameters and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // connect whichever pipeline is desired and comment the other one
        camera.setPipeline(contourPipeline);
        //camera.setPipeline(yellowPipeline);

        // runs camera on a separate thread so it can run simultaneously with everything else
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                // starts the camera stream when init is pressed
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public ContourPipeline getContourPipeline() {
        return contourPipeline;
    }
}
