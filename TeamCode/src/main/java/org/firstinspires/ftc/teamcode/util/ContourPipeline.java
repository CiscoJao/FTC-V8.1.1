package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
AVERAGE PROCESSING TIMES:
320x176 30 ms
800x448 60-80 ms
1280x960 220 ms
This pipeline is designed to filter out the yellow poles and detect their edges.
Additionally, the pipeline also finds the areas and their center of masses of the detected
yellow shapes.
Created by Francisco Jao
 */
public class ContourPipeline extends OpenCvPipeline {

    // for tracking pipeline processing speed
    private ElapsedTime timer = new ElapsedTime();
    private double processTime = 0;

    // constants
    private final Rect LEFT_RECT = new Rect(1, 1, 300, 447);
    private final Rect MID_RECT = new Rect(301, 1, 500, 447);
    private final Rect RIGHT_RECT = new Rect(501, 1, 799, 447);
    private final Size KERNEL = new Size(20, 20);
    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar CONTOUR_COLOR = new Scalar(0, 255, 255);
    private final Scalar CONTOUR_CENTER_COLOUR = new Scalar(255,0,255);
    private final Scalar UPPER_HSV = new Scalar(30, 255, 255);
    private final Scalar LOWER_HSV = new Scalar(10, 130, 130);
    private final double LOWER_HYSTERESIS = 1.0;
    private final double UPPER_HYSTERESIS = 2.0;
    private final int APERTURE = 3;

    // MATRICES
    Mat leftSub = new Mat();
    Mat middleSub = new Mat();
    Mat rightSub = new Mat();
    Mat output = new Mat();

    // for contour detection
    private List<Double> contourAreas = new ArrayList<>(); // records contour areas in pixels
    private double largestContourArea = 0;
    private Point largestContourCenter = new Point(0, 0);

    @Override
    public Mat processFrame(Mat input) {

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

        // finding contours (different from Canny Edge detection)
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        // RETR_EXTERNAL means to retrieve only the external contours
        // CHAIN_APPROX_SIMPLE means to compress horizontal, vertical, and diagonal segments and only leaves their endpoints
        Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // set up output image where contours will be drawn on (for testing)
        Mat draw = Mat.zeros(threshold.size(), CvType.CV_8UC3);

        // record all of the contour areas found in a list
        contourAreas.clear();
        if (!contours.isEmpty()) {
            for (int i = 0; i < contours.size(); i++) {
                contourAreas.add(Imgproc.contourArea(contours.get(i)));

                // using properties of image moments to calculate the center of masses of contours (from docs.opencv.org)
                Moments currMoments = Imgproc.moments(contours.get(i));
                double cx = currMoments.get_m10() / currMoments.get_m00();
                double cy = currMoments.get_m01() / currMoments.get_m00();

                // draw the contour centers on output image
                Imgproc.drawMarker(draw, new Point(cx, cy), CONTOUR_CENTER_COLOUR);
            }

            // draw all of the contours on output image
            Imgproc.drawContours(draw, contours, -1, CONTOUR_COLOR);

            // finding the largest contour area and the location of its center
            largestContourArea = Collections.max(contourAreas, null);
            findLargestContourCenter(contours);
        }

        input.copyTo(output);

        // drawing three rectangles on camera feed
//        Imgproc.rectangle(output, RIGHT_RECT, WHITE, 1);
//        Imgproc.rectangle(output, MID_RECT, WHITE, 1);
//        Imgproc.rectangle(output, LEFT_RECT, WHITE, 1);

        // deallocating matrix memory
        blur.release();
        hsv.release();
        threshold.release();
        edges.release();
        hierarchy.release();
        draw.release();

        // tracking pipeline speed
        processTime = timer.milliseconds();
        timer.reset();

        return output; // what the camera stream will display on stream
    }

    /*
     * finds the index of the largest contour and uses that index to grab the desired contour
     * from the list, then it grabs the image moment properties to calculate its enter of mass
     *
     * although the code for this is very long, it is the method that takes up the least memory
    */
    private void findLargestContourCenter(List<MatOfPoint> contours) {
        largestContourCenter.x = Imgproc.moments(contours.get(contourAreas.indexOf(largestContourArea))).get_m10() /
                Imgproc.moments(contours.get(contourAreas.indexOf(largestContourArea))).get_m00();
        largestContourCenter.y = Imgproc.moments(contours.get(contourAreas.indexOf(largestContourArea))).get_m01() /
                Imgproc.moments(contours.get(contourAreas.indexOf(largestContourArea))).get_m00();
    }

    public double getProcessTime() {
        return processTime;
    }

    public List<Double> getContourAreas() {
        return contourAreas;
    }

    public double largestContourArea() {
        return largestContourArea;
    }

    public Point largestContourCenter() {
        return largestContourCenter;
    }

    public boolean poleDetected() {
        return !contourAreas.isEmpty();
    }
}
