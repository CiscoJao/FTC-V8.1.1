package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PoleFollowPipeline extends OpenCvPipeline {

    // CONSTANTS
    private final Size KERNEL = new Size(20, 20);
    private final Scalar CONTOUR_COLOUR = new Scalar(0, 255, 255);
    private final Scalar CONTOUR_CENTER_COLOUR = new Scalar(255, 0, 255);
    private final Scalar UPPER_HSV = new Scalar(30, 255, 255);
    private final Scalar LOWER_HSV = new Scalar(10, 130, 130);

    // final output image
    Mat output = new Mat();

    // for contour detection
    private List<Double> contourAreas = new ArrayList<>();
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

        // finding contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        // RETR_EXTERNAL means to retrieve only the external contours
        // CHAIN_APPROX_SIMPLE means to compress horizontal,
        // vertical, and diagonal segments and only leaves their endpoints
        Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // set up output image where contours will be drawn on (creates a pure black image)
        Mat draw = Mat.zeros(threshold.size(), CvType.CV_8UC3);

        // record all of the contour areas found in a list
        contourAreas.clear();
        if (!contours.isEmpty()) {
            for (int i = 0; i < contours.size(); i++) {
                contourAreas.add(Imgproc.contourArea(contours.get(i)));

                // using properties of image moments to calculate the
                // center of masses of contours (from docs.opencv.org)
                Moments currMoments = Imgproc.moments(contours.get(i));
                double cx = currMoments.get_m10() / currMoments.get_m00();
                double cy = currMoments.get_m01() / currMoments.get_m00();

                // draw the contour centers on output image
                Imgproc.drawMarker(draw, new Point(cx, cy), CONTOUR_CENTER_COLOUR);
            }

            // draw all of the contours on output image
            Imgproc.drawContours(draw, contours, -1, CONTOUR_COLOUR);

            // finding the largest contour area and the location of its center
            largestContourArea = Collections.max(contourAreas, null);
            findLargestContourCenter(contours);
        }

        draw.copyTo(output);

        // deallocate matrix memory
        blur.release();
        hsv.release();
        threshold.release();
        hierarchy.release();
        draw.release();

        return output;
    }

    /*
     * finds the index of the largest contour and uses that index to grab the desired contour
     * from the list, then it grabs the image moment properties to calculate its enter of mass
     */
    private void findLargestContourCenter(List<MatOfPoint> contours) {
        MatOfPoint largestContour = contours.get(contourAreas.indexOf(largestContourArea));

        largestContourCenter.x = Imgproc.moments(largestContour).get_m10() / Imgproc.moments(largestContour).get_m00();
        largestContourCenter.y = Imgproc.moments(largestContour).get_m01() / Imgproc.moments(largestContour).get_m00();
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
