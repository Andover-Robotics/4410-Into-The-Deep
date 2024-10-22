package org.firstinspires.ftc.teamcode.auto.pipelines;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
public class ChamberDetectorTesting extends OpenCvPipeline {
    Telemetry telemetry;
    public static int minWidth = 60; // TODO: Need to figure out minWidth;
    public static int camWidth = 1280;
    // variables for distance equation, found online for c270:
    public static int camHeight = 720;
    public static double realChamberHeight = 19.05; // Real height of chamber in mm
    public static double focalLength = 4.0; // f(mm) of c270
    public static double sensorHeight = 3.58; // height of lens in mm

    public enum ChamberDetected {
        DETECTED,
        NOTDETECTED,
        HIGH,
        LOW
    }
    public static ChamberDetected chamberDetected = ChamberDetected.NOTDETECTED;
    public static double lowH = 237, lowS = 93, lowV = 70, highH = 237, highS = 93, highV = 100;
    // TODO: Base values - need to actually figure out HSV values for low and high blue and red
    public static Scalar blueLowHSV = new Scalar(lowH, lowS, lowV);
    public static Scalar blueHighHSV = new Scalar(highH, highS, highV);
    Mat HSV = new Mat();
    MatOfPoint big;


    public ChamberDetectorTesting(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Rect rectangle = new Rect(0, 360, 720, 10);
        // TODO: Need to figure out dimensions of rectangle

        Imgproc.rectangle(input, rectangle, new Scalar(240, 100, 50), 5);
        // TODO: Need to figure out HSV for rectangle

        Core.inRange(HSV, blueLowHSV, blueHighHSV, HSV);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(HSV, contours, new Mat(), 0, 1);

        if (!contours.isEmpty()) {
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).height)));

            big = contours.get(0);
            // Who's the most gargantuan rectangle ?

            Rect rect = Imgproc.boundingRect(big);

            if (rect.width > minWidth) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 6);

                double midRect = rect.tl().y + rect.height / 2.0;

                if (midRect > rectangle.tl().y && midRect < rectangle.br().y) {
                    chamberDetected = ChamberDetected.DETECTED;
                } else if (midRect < rectangle.tl().y){
                    chamberDetected = ChamberDetected.HIGH;
                } else if (midRect > rectangle.br().y) {
                    chamberDetected = ChamberDetected.LOW;
                } else {
                    chamberDetected = ChamberDetected.NOTDETECTED;
                } // Check where the skibidi rectangle is in relation to where it should be
                telemetry.addData("Midpoint of bounding box: ", midRect);
            } else {
                chamberDetected = ChamberDetected.NOTDETECTED;
            }
        } else {
            chamberDetected = ChamberDetected.NOTDETECTED;
        }
        telemetry.addData("Chamber Detection Status: ", chamberDetected);

        HSV.release();

        return input;
    }
    // Find the distance with equation i found on youtube
    public double getDistance() {
        Rect rect = Imgproc.boundingRect(big);
        return (focalLength * realChamberHeight * camHeight)/(rect.height * sensorHeight);
        // Equation to find distance
    }
}
