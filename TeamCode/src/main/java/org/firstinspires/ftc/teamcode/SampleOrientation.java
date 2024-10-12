package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.List;
import java.util.ArrayList;

public class SampleOrientation implements VisionProcessor {
    private Mat Frame = new Mat();
    private Mat HsvMat = new Mat();
    private Mat RedMask = new Mat();
    private Mat BlueMask = new Mat();
    private Mat YellowMask = new Mat();

    private List<MatOfPoint> RedContours = new ArrayList<>();
    private List<MatOfPoint> BlueContours = new ArrayList<>();
    private List<MatOfPoint> YellowContours = new ArrayList<>();

    private final Scalar redHSV = new Scalar(0,175,127);
    private final Scalar blueHSV = new Scalar(231/2,125,127);
    private final Scalar yellowHSV = new Scalar(49.5/2,175,127);

    private final Scalar redRange = new Scalar(5,79,127);
    private final Scalar blueRange = new Scalar(10,80,127);
    private final Scalar yellowRange = new Scalar(5,79,127);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
        Frame = frame;

        Core.inRange(HsvMat,range(redHSV, redRange, false),range(redHSV, redRange, true),RedMask);
        Core.inRange(HsvMat,range(blueHSV, blueRange, false),range(blueHSV, blueRange, true),BlueMask);
        Core.inRange(HsvMat,range(yellowHSV, yellowRange, false),range(yellowHSV, yellowRange, true),YellowMask);

        Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(BlueMask, BlueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
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
}
