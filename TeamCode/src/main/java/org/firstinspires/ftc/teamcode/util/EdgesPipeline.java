package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
This pipeline is designed to detect the edges of yellow poles using the Canny edge detection algorithm.
The algorithm is frequently used with blurred images to remove unwanted noise.

NOTE: Edge detection is NOT the same as finding contours. Contours are recognized as individual shapes and
have geometric properties that OpenCV works with. Edges are merely detecting contrast gradients in images
and have no recognizable geometric significance.
 */

public class EdgesPipeline extends OpenCvPipeline {

    // for tracking pipeline processing speed
    private ElapsedTime timer = new ElapsedTime();
    private double processTime = 0;

    // constants
    private final Size KERNEL = new Size(20, 20);
    private final Scalar UPPER_HSV = new Scalar(30, 255, 255);
    private final Scalar LOWER_HSV = new Scalar(10, 130, 130);
    private final double LOWER_HYSTERESIS = 1.0;
    private final double UPPER_HYSTERESIS = 2.0;
    private final int APERTURE = 3;

    @Override
    public Mat processFrame(Mat input) {

        // final output image
        Mat output = new Mat();

        // blurring the input image to improve edge and contour detection
        Mat blur = new Mat();
        Imgproc.blur(input, blur, KERNEL);

        // convert blurred image into HSV scale
        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        // create a HSV threshold that filters out yellow shades
        Mat threshold = new Mat();
        Core.inRange(hsv, LOWER_HSV, UPPER_HSV, threshold);

        // perform Canny edge detection algorithm on the threshold binary image
        // this will create neat lines around the poles
        Mat edges = new Mat();
        Imgproc.Canny(threshold, edges, UPPER_HYSTERESIS, LOWER_HYSTERESIS, APERTURE);

        edges.copyTo(output);

        // deallocating matrix memory
        blur.release();
        hsv.release();
        threshold.release();
        edges.release();

        // tracking pipeline speed
        processTime = timer.milliseconds();
        timer.reset();

        return output;
    }

    private double getProcessTime() {
        return processTime;
    }
}
