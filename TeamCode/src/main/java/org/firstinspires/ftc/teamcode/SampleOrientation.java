package org.firstinspires.ftc.teamcode;

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

    private RotatedRect RedRect = new RotatedRect();
    private RotatedRect BlueRect = new RotatedRect();
    private RotatedRect YellowRect = new RotatedRect();

    private List<MatOfPoint> RedContours = new Vector<>();
    private List<MatOfPoint> BlueContours = new Vector<>();
    private List<MatOfPoint> YellowContours = new Vector<>();

    private final static Scalar RED_HSV = new Scalar(0,175,127);
    private final static Scalar BLUE_HSV = new Scalar(231/2,125,127);
    private final static Scalar YELLOW_HSV = new Scalar(49.5/2,175,127);

    private final static Scalar RED_RANGE = new Scalar(5,79,127);
    private final static Scalar BLUE_RANGE = new Scalar(10,80,127);
    private final static Scalar YELLOW_RANGE = new Scalar(5,79,127);

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
        if (alliance == Alliance.ALLIANCE_RED) {
            // red alliance detection system
            Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
            Frame = frame;

            Core.inRange(HsvMat, range(RED_HSV, RED_RANGE, false), range(RED_HSV, RED_RANGE, true), RedMask);
            Core.inRange(HsvMat, range(YELLOW_HSV, YELLOW_RANGE, false), range(YELLOW_HSV, YELLOW_RANGE, true), YellowMask);

            Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f RedMat = new MatOfPoint2f; RedMat.fromList(largestContour(RedContours));
            MatOfPoint2f YellowMat = new MatOfPoint2f; YellowMat.fromList(largestContour(YellowContours));

            RedRect = Imgproc.minAreaRect(RedMat);
            YellowRect = Imgproc.minAreaRect(YellowMat);

        } else {
            // blue alliance detection system
            Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
            Frame = frame;

            Core.inRange(HsvMat, range(BLUE_HSV, BLUE_RANGE, false), range(BLUE_HSV, BLUE_RANGE, true), BlueMask);
            Core.inRange(HsvMat, range(YELLOW_HSV, YELLOW_RANGE, false), range(YELLOW_HSV, YELLOW_RANGE, true), YellowMask);

            Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f BlueMat = new MatOfPoint2f; BlueMat.fromList(largestContour(RedContours));
            MatOfPoint2f YellowMat = new MatOfPoint2f; YellowMat.fromList(largestContour(YellowContours));

            BlueRect = Imgproc.minAreaRect(BlueMat);
            YellowRect = Imgproc.minAreaRect(YellowMat);

        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Mat matCanvas = Frame;

        Imgproc.drawContours(matCanvas, RedContours, -1, new Scalar(255,0,0), 5);
        Imgproc.drawContours(matCanvas, BlueContours, -1, new Scalar(0,0,255), 5);
        Imgproc.drawContours(matCanvas, YellowContours, -1, new Scalar(240,240,0), 5);

        RedContours.clear(); BlueContours.clear(); YellowContours.clear();

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
        assert largest != null;
        return largest.toList();
    }
}
