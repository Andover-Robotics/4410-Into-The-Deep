package org.firstinspires.ftc.teamcode.vision;

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
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.List;
import java.util.Vector;

public class SampleTrackingPipeline extends OpenCvPipeline implements VisionProcessor {

    public enum Alliance {
        BLUE,
        RED,
        NULL,
    }

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

    private final static Scalar RED_LOW_HSV = new Scalar(0,60,60);
    private final static Scalar BLUE_LOW_HSV = new Scalar(105,60,60);
    private final static Scalar YELLOW_LOW_HSV = new Scalar(15,60,60);

    private final static Scalar RED_HIGH_HSV = new Scalar(4, 253, 253);
    private final static Scalar BLUE_HIGH_HSV = new Scalar(115, 255, 255);
    private final static Scalar YELLOW_HIGH_HSV = new Scalar(25, 255, 255);

    private Point[] RedLine = new Point[2];
    private Point[] BlueLine = new Point[2];
    private Point[] YellowLine = new Point[2];

    private double angleRed = -4410, angleBlue = -4410, angleYellow = -4410;

    public boolean detectYellow;

    public Alliance alliance = Alliance.NULL;

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

    public double getAngleBlue() {
        return angleBlue;
    }

    public double getAngleYellow() {
        return angleYellow;
    }

    public Alliance getAlliance() { return alliance; }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Mat processFrame(Mat input) {
        RedContours.clear(); BlueContours.clear(); YellowContours.clear();

        Imgproc.cvtColor(input, HsvMat, Imgproc.COLOR_RGB2HSV);
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
            Core.inRange(HsvMat, BLUE_LOW_HSV, BLUE_HIGH_HSV, BlueMask);
            Imgproc.findContours(BlueMask, BlueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!BlueContours.isEmpty()) {
                LargestBlue.fromList(largestContour(BlueContours));

                try {
                    BlueRect = Imgproc.minAreaRect(LargestBlue);

                    Point[] BlueRectPoints = new Point[4]; BlueRect.points(BlueRectPoints);
                    Point[] BlueLongestLine = findLongestSide(BlueRectPoints[1],BlueRectPoints[2],BlueRectPoints[3]);

                    BlueLine = BlueLongestLine;

                    angleBlue = getAngle(BlueLongestLine[0],BlueLongestLine[1]);
                } finally {}
            }
        }

        if (detectYellow) {
            Core.inRange(HsvMat, YELLOW_LOW_HSV, YELLOW_HIGH_HSV, YellowMask);
            Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!YellowContours.isEmpty()) {
                LargestYellow.fromList(largestContour(YellowContours));

                try {
                    YellowRect = Imgproc.minAreaRect(LargestYellow);

                    Point[] YellowRectPoints = new Point[4]; YellowRect.points(YellowRectPoints);
                    Point[] YellowLongestLine = findLongestSide(YellowRectPoints[1],YellowRectPoints[2],YellowRectPoints[3]);

                    YellowLine = YellowLongestLine;

                    angleYellow = getAngle(YellowLongestLine[0],YellowLongestLine[1]);
                } finally {}
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

        // Convert Mat to Bitmap
        Bitmap bitmap = Bitmap.createBitmap(matCanvas.cols(), matCanvas.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(matCanvas, bitmap);

        canvas.drawBitmap(bitmap, 0, 0, null);
    }

    private List<Point> largestContour(List<MatOfPoint> contours) {
        double area = 3500;
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
        Point point1 = new Point();
        Point point2 = new Point();

        if (p1.x < p2.x) {
            point1 = p1;
            point2 = p2;
        } else {
            point1 = p2;
            point2 = p1;
        }
        return (Math.toDegrees(Math.atan2(point2.y-point1.y,point2.x-point1.x)));
    }

    private Point[] findLongestSide(Point p1, Point p2, Point p3) {
        double[] lengths = new double[2];
        Point[] longestPair = new Point[2];
        lengths[0] = Math.sqrt(Math.pow(p1.x-p2.x,2)+Math.pow(p1.y-p2.y,2));
        lengths[1] = Math.sqrt(Math.pow(p2.x-p3.x,2)+Math.pow(p2.y-p3.y,2));

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
