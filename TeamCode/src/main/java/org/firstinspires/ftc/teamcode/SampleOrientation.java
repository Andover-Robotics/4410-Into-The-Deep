package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
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

    private final static Scalar RED_LOW_HSV = new Scalar(0,190,168);
    private final static Scalar BLUE_LOW_HSV = new Scalar(115,190,168);
    private final static Scalar YELLOW_LOW_HSV = new Scalar(25,190,168);

    private final static Scalar RED_HIGH_HSV = new Scalar(8, 253, 253);
    private final static Scalar BLUE_HIGH_HSV = new Scalar(125, 270, 295);
    private final static Scalar YELLOW_HIGH_HSV = new Scalar(35, 253, 253);

    private Point[] RedLine = new Point[2];

    private double angleRed, angleBlue, angleYellow;

    private boolean detectYellow;

    private enum Alliance {
        BLUE,
        RED
    }

    private Alliance alliance;

    public void setBlueAlliance() {
        alliance = Alliance.BLUE;
    }

    public void setRedAlliance() {
        alliance = Alliance.RED;
    }

    public void detectYellow(boolean set) {
        detectYellow = set;
    }

    public double getAngleRed() {
        return angleRed;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        RedContours.clear(); BlueContours.clear(); YellowContours.clear();

        Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(HsvMat, HsvMat, 5);
        Imgproc.cvtColor(HsvMat, Frame, Imgproc.COLOR_HSV2RGB);

        if (alliance == Alliance.RED) {
            // red alliance detection system
            Core.inRange(HsvMat, RED_LOW_HSV, RED_HIGH_HSV, RedMask);
            Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!RedContours.isEmpty()) {
                LargestRed.fromList(largestContour(RedContours));

                try {
                    RedRect = Imgproc.minAreaRect(LargestRed);

                    Point[] RedRectPoints = new Point[4]; RedRect.points(RedRectPoints);
                    Point[] RedLongestLine = findLongestSide(RedRectPoints[1],RedRectPoints[2],RedRectPoints[3]);

                    RedLine = RedLongestLine;

                    angleRed = getAngle(RedLongestLine[0],RedLongestLine[1]);
                } finally {}
            }
        } else {
            // blue samples
            Core.inRange(HsvMat, BLUE_LOW_HSV, BLUE_HIGH_HSV, YellowMask);
            Imgproc.findContours(BlueMask, BlueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            LargestBlue.fromList(largestContour(BlueContours));

            try {
                BlueRect = Imgproc.minAreaRect(LargestBlue);

                Point[] RedRectPoints = new Point[4]; BlueRect.points(RedRectPoints);
                //a1 = getAngle(RedRectPoints[0],RedRectPoints[1]);
                //a2 = getAngle(RedRectPoints[1],RedRectPoints[2]);
                //a3 = getAngle(RedRectPoints[2],RedRectPoints[3]);
                //a4 = getAngle(RedRectPoints[3],RedRectPoints[0]);
            } finally {}
        }

        if (detectYellow) {
            Core.inRange(HsvMat, YELLOW_LOW_HSV, YELLOW_HIGH_HSV, YellowMask);
            Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!YellowContours.isEmpty()) {
                LargestYellow.fromList(largestContour(YellowContours));


                try {
                    YellowRect = Imgproc.minAreaRect(LargestYellow);

                    Point[] RedRectPoints = new Point[4]; YellowRect.points(RedRectPoints);
                    //a1 = getAngle(RedRectPoints[0],RedRectPoints[1]);
                    //a2 = getAngle(RedRectPoints[1],RedRectPoints[2]);
                    //a3 = getAngle(RedRectPoints[2],RedRectPoints[3]);
                    //a4 = getAngle(RedRectPoints[3],RedRectPoints[0]);
                } finally {
                }
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Mat matCanvas = Frame;

        //Imgproc.drawContours(matCanvas, RedContours, -1, new Scalar(255,0,0), 5);
        //Imgproc.drawContours(matCanvas, BlueContours, -1, new Scalar(0,0,255), 5);
        //Imgproc.drawContours(matCanvas, YellowContours, -1, new Scalar(240,240,0), 5);

        if (alliance == Alliance.RED) {
            try {
                Point[] RedCorners = new Point[4];
                RedRect.points(RedCorners);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(matCanvas, RedCorners[i], RedCorners[(i + 1) % 4], new Scalar(255, 0, 0), 5);
                    Imgproc.putText(matCanvas, String.valueOf(i), RedCorners[i],FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,255,255));
                }

            } finally {
            }
        } else {
            try {
                Point[] BlueCorners = new Point[4];
                BlueRect.points(BlueCorners);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(matCanvas, BlueCorners[i], BlueCorners[(i + 1) % 4], new Scalar(0, 0, 255), 5);
                }
            } finally {
            }
        }

        if (detectYellow) {
            try {
                Point[] YellowCorners = new Point[4];
                YellowRect.points(YellowCorners);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(matCanvas, YellowCorners[i], YellowCorners[(i + 1) % 4], new Scalar(240, 240, 0), 5);
                }
            } finally {
            }
        }


        Imgproc.line(matCanvas, RedLine[0], RedLine[1], new Scalar(225, 225, 225), 5);


        // Convert Mat to Bitmap
        Bitmap bitmap = Bitmap.createBitmap(matCanvas.cols(), matCanvas.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(matCanvas, bitmap);

        canvas.drawBitmap(bitmap, 0, 0, null);
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
            return java.util.Collections.emptyList();
        }
    }

    private double getAngle (Point p1, Point p2) {
        return Math.toDegrees(Math.atan2(Math.abs(p2.y-p1.y),p2.x-p1.x));
    }

    private Point[] findLongestSide(Point p1, Point p2, Point p3) {
        double[] lengths = new double[2];
        Point[] longestPair = new Point[2];
        lengths[0] = Math.sqrt(Math.pow(p1.x-p2.x,2)+Math.pow(p1.x-p2.x,2));
        lengths[1] = Math.sqrt(Math.pow(p2.x-p3.x,2)+Math.pow(p2.x-p3.x,2));

        if (lengths[0] > lengths[1]) {
            longestPair[0] = p1;
            longestPair[1] = p2;
        } else {
            longestPair[0] = p2;
            longestPair[1] = p3;
        }
        return longestPair;
    }
}
