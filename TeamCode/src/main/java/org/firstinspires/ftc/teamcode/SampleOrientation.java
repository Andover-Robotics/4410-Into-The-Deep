package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint2f;


import java.util.List;
import java.util.Vector;

public class SampleOrientation implements VisionProcessor {
    private Mat Frame = new Mat();
    private Mat HsvMat = new Mat();
    private Mat RedMask = new Mat();
    private Mat BlueMask = new Mat();
    private Mat YellowMask = new Mat();

    private MatOfPoint2f LargestRed = new MatOfPoint2f();
    private MatOfPoint2f LargestYellow = new MatOfPoint2f();
    private MatOfPoint2f LargestBlue = new MatOfPoint2f();

    private RotatedRect RedRect = new RotatedRect();
    private RotatedRect BlueRect = new RotatedRect();
    private RotatedRect YellowRect = new RotatedRect();

    private List<MatOfPoint> RedContours = new Vector<>();
    private List<MatOfPoint> BlueContours = new Vector<>();
    private List<MatOfPoint> YellowContours = new Vector<>();

    private final static Scalar RED_HSV = new Scalar(0,190,168);
    private final static Scalar BLUE_HSV = new Scalar(231/2,190,168);
    private final static Scalar YELLOW_HSV = new Scalar(25,190,127);

    private final static Scalar RED_RANGE = new Scalar(5,63,85);
    private final static Scalar BLUE_RANGE = new Scalar(10,80,127);
    private final static Scalar YELLOW_RANGE = new Scalar(10,63,85);

    private double a1, a2, a3, a4;

    private enum Alliance {
        ALLIANCE_BLUE,
        ALLIANCE_RED
    }

    private Alliance alliance;

    public void setBlueAlliance() {
        alliance = Alliance.ALLIANCE_BLUE;
    }

    public void setRedAlliance() {
        alliance = Alliance.ALLIANCE_RED;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        RedContours.clear(); BlueContours.clear(); YellowContours.clear();
        if (alliance == Alliance.ALLIANCE_RED) {
            // red alliance detection system
            Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
            Imgproc.medianBlur(HsvMat, HsvMat, 5); // remove salt and pepper noise;
            //Imgproc.GaussianBlur(HsvMat,HsvMat, new Size(7,7),0);//remove gaussian noise
            Imgproc.cvtColor(HsvMat, Frame, Imgproc.COLOR_HSV2RGB);

            Core.inRange(HsvMat, range(RED_HSV, RED_RANGE, false), range(RED_HSV, RED_RANGE, true), RedMask);
            Core.inRange(HsvMat, range(YELLOW_HSV, YELLOW_RANGE, false), range(YELLOW_HSV, YELLOW_RANGE, true), YellowMask);

            Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            boolean contoursDetected = true;

            LargestRed.fromList(largestContour(RedContours));
            LargestYellow.fromList(largestContour(YellowContours));

            try {
                RedRect = Imgproc.minAreaRect(LargestRed);
                YellowRect = Imgproc.minAreaRect(LargestYellow);

                Point[] RedRectPoints = new Point[4]; RedRect.points(RedRectPoints);
                a1 = getAngle(RedRectPoints[0],RedRectPoints[1]);
                a2 = getAngle(RedRectPoints[1],RedRectPoints[2]);
                a3 = getAngle(RedRectPoints[2],RedRectPoints[3]);
                a4 = getAngle(RedRectPoints[3],RedRectPoints[0]);
            } finally {
            }

        } else {
            // blue alliance detection system

        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Mat matCanvas = Frame;

        Imgproc.drawContours(matCanvas, RedContours, -1, new Scalar(255,0,0), 5);
        Imgproc.drawContours(matCanvas, BlueContours, -1, new Scalar(0,0,255), 5);
        Imgproc.drawContours(matCanvas, YellowContours, -1, new Scalar(240,240,0), 5);

        try {
            Point[] RedCorners = new Point[4]; RedRect.points(RedCorners);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(matCanvas, RedCorners[i], RedCorners[(i + 1) % 4], new Scalar(255,0,0), 5);
            }
        } finally {
        }

        // Convert Mat to Bitmap
        Bitmap bitmap = Bitmap.createBitmap(matCanvas.cols(), matCanvas.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(matCanvas, bitmap);

        canvas.drawBitmap(bitmap, 0, 0, null);
    }

    // helper function for ranges because i'm a lazy ass
    private Scalar range(Scalar input, Scalar range, boolean retUpper) {
        Scalar ret = new Scalar(0,0,0);
        if (retUpper) {
            ret = new Scalar (
                    Math.max((input.val[0] + range.val[0])%255, 0.0),
                    Math.max((input.val[1] + range.val[1])%255, 0.0),
                    Math.max((input.val[2] + range.val[2])%255, 0.0)
            );
        } else {
            ret = new Scalar (
                    Math.max((input.val[0] - range.val[0])%255, 0.0),
                    Math.max((input.val[1] - range.val[1])%255, 0.0),
                    Math.max((input.val[2] - range.val[2])%255, 0.0)
            );
        }
        return ret;
    }

    private List<Point> largestContour(List<MatOfPoint> contours) {
        double area = -69420;
        MatOfPoint largest = null;
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > area) {
                area = (Imgproc.contourArea(contour));
                largest = contour;
            }
        }
        if (largest != null) {
            return largest.toList();
        } else {
            return null;
        }
    }

    private double getAngle (Point p1, Point p2) {
        return Math.atan2(Math.abs(p1.y-p1.y),Math.abs(p1.x-p1.x));
    }

    public double getAngle(int input){
        switch(input) {
            case 1:
                return a1;
            case 2:
                return a2;
            case 3:
                return a3;
            case 4:
                return a4;
        }
        return 0;
    }
}
