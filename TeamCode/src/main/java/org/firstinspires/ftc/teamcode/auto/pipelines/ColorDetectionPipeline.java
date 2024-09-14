package org.firstinspires.ftc.teamcode.auto.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/*
TODO: FIX WHATEVER MESS OF CODE THIS IS SOME OTHER TIME AFTER SCRIMMAGE - ZACHERY
 */

public class ColorDetectionPipeline extends OpenCvPipeline{

    Telemetry telemetry;
    Mat HSV = new Mat();
    MatOfPoint biggest;

    public static int minheight = 100;
    public static int minwidth = 100;
    public static int width = 0;
    public static int height = 0;
    public static int camwidth = 1280;
    public static int camheight = 720;

    public static double midpointrect;

    public enum SpikeMark{
        LEFT,
        MIDDLE,
        RIGHT,
        NOTDETECTED
    }
    public static SpikeMark spikeMark = SpikeMark.NOTDETECTED;

    // Alliance: 0 - NONE, 1 - RED, 2 - BLUE
    public static int alliance = 0;

    //
    public static int centerTapeMinHeight = 55;

    // Red HSV Values
    public static double redLH = 0, redLS = 56, redLV = 35, redHH = 20, redHS = 360, redHV = 176;
    public static Scalar redLowHSV= new Scalar(redLH,redLS,redLV);
    public static Scalar redHighHSV = new Scalar(redHH,redHS,redHV);
//    public static Scalar redLowHSV2= new Scalar(redLH2,redLS,redLV);
//    public static Scalar redHighHSV2 = new Scalar(redHH2,redHS,redHV);

    // Blue HSV Values
    public static double blueLH = 90, blueLS = 56, blueLV = 35, blueHH = 140, blueHS = 360, blueHV = 176;
    public static Scalar blueLowHSV= new Scalar(blueLH,blueLS,blueLV);
    public static Scalar blueHighHSV = new Scalar(blueHH,blueHS,blueHV);

    public static Scalar currentHighHSV = redHighHSV;
    public static Scalar currentLowHSV = redLowHSV;
    public static int count = 0;

    public ColorDetectionPipeline(Telemetry telemetry){ // CONSTRUCTOR :D
        spikeMark = SpikeMark.NOTDETECTED;
        this.telemetry = telemetry;
    }

    public void setAlliance(int alliance) {
        ColorDetectionPipeline.alliance = alliance;
//        if (alliance == 1) {
//            currentHighHSV = redHighHSV;
//            currentLowHSV = redLowHSV;
//        } else {
//            currentHighHSV = blueHighHSV;
//            currentLowHSV = blueLowHSV;
//        }
    }

    public void setCenterTapeHeight (int height) {
        centerTapeMinHeight = height;
    }
    public int getCenterTapeHeight() {
        return centerTapeMinHeight;
    }


    public int getSpikeMark() {
        switch (spikeMark) {
            case LEFT: return 1;
            case MIDDLE: return 2;
            case RIGHT: return 3;
        }
        return 0; // None detected
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(600, 50, 680, 240);
        Rect leftrect = new Rect(100, 0, 325, 400); // rectangle sizes

        Imgproc.rectangle(input, leftrect, new Scalar(0, 255, 0), 5); //displays rectangles with red color
        Imgproc.rectangle(input, rightrect, new Scalar(0, 255, 0), 5);

        if (alliance == 1) {
            Core.inRange(HSV, redLowHSV, redHighHSV, HSV);
//            redHighHSV = currentHighHSV;
//            redLowHSV = currentLowHSV;
//            Core.inRange(HSV, redLowHSV2, redHighHSV2, HSV);
        } else {
            Core.inRange(HSV, blueLowHSV, blueHighHSV, HSV);
//            blueHighHSV = currentHighHSV;
//            redLowHSV = currentLowHSV;
        }

        // filters HSV mat into image with black being the lowest red/blue HSV and white being the highest red/blue HSV
//        if (alliance == 1) {
//            if (count == 0) {
//                currentLowHSV = redLowHSV;
//                currentHighHSV = redHighHSV;
//            } else {
//                redLowHSV = currentLowHSV;
//                redHighHSV = currentHighHSV;
//            }
//
//            Core.inRange(HSV, redLowHSV, redHighHSV, HSV);
//
//
//            //Core.inRange(HSV, redLowHSV2, redHighHSV2, HSV);
//        } else {
//            if (count == 0) {
//                currentLowHSV = blueLowHSV;
//                currentHighHSV = blueHighHSV;
//            } else {
//                blueLowHSV = currentLowHSV;
//                blueHighHSV = currentHighHSV;
//            }
//
//            Core.inRange(HSV, blueLowHSV, blueHighHSV, HSV);
//
//        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(HSV, contours, new Mat(), 0,1); // finds contours in HSV mat

        if (!contours.isEmpty()) { // checks if no contours are found
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)

            // Make rectangle
            int iterations = 0;
            biggest = contours.get(0); //contour with the largest width(first in the array)
            Rect rect = Imgproc.boundingRect(biggest); // turns biggest contour into a rectangle
            midpointrect = rect.tl().x + rect.width/2.0; // gets midpoint x of the rectangle
            width = rect.width;
            height = rect.height;
            while (height < centerTapeMinHeight) {
                if (iterations < contours.size()) {
                    biggest = contours.get(iterations);
                    rect = Imgproc.boundingRect(biggest); // turns biggest contour into a rectangle
                    midpointrect = rect.tl().x + rect.width / 2.0; // gets midpoint x of the rectangle
                    width = rect.width;
                    height = rect.height;
                    iterations++;
                } else {
                    spikeMark = SpikeMark.RIGHT;
                    break;
                }
            }

            // Draw rectangle on screen
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 6); // puts border around contours with a green shade

            // Check which zone it is in (left or right)
            if (midpointrect > leftrect.tl().x && midpointrect < leftrect.br().x) { // LEFT SPIKE
                if (height < 190|| height > 770) { // make sure it is not left tape
                    spikeMark = SpikeMark.RIGHT;
                } else {
                    spikeMark = SpikeMark.LEFT;
                }
            } else if (midpointrect > rightrect.tl().x && midpointrect < leftrect.br().x) {
                if (height > centerTapeMinHeight) { // make sure it is not center tape
                    spikeMark = SpikeMark.MIDDLE;
                } else {
                    spikeMark = SpikeMark.RIGHT;
                }
            } else {
                if (height > centerTapeMinHeight) { // make sure it is not center tape
                    spikeMark = SpikeMark.MIDDLE;
                } else {
                    spikeMark = SpikeMark.RIGHT;
                }
            }
            //telemetry.addLine("Midpoint of Bounding Box :"+ midpointrect);
        } else { // NOT IN FRAME, RIGHT SPIKE
            spikeMark = SpikeMark.RIGHT;
        }
        //telemetry.addData("contours: ", contours.size());
        // telemetry.addData("Spikemark status: ",spikeMark);
        // Releasing all our mats for the next iteration
        HSV.release();
        count++;
        return input; // return end frame with rectangles drawn
    }
}