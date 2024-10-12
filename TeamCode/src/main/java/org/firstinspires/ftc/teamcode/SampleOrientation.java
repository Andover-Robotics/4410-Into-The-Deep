package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class SampleOrientation implements VisionProcessor {

    private Mat HsvMat = new Mat();
    private Mat RedMask = new Mat();
    private Mat BlueMask = new Mat();
    private Mat YellowMask = new Mat();

    private List<MatOfPoint> RedContours = new ArrayList<>();
    private java.util.List<MatOfPoint> BlueContours = new ArrayList<>();
    private java.util.List<MatOfPoint> YellowContours = new ArrayList<>();

    private final Scalar redHSV = new Scalar(0,0,0);
    private final Scalar blueHSV = new Scalar(0,0,0);
    private final Scalar yellowHSV = new Scalar(0,0,0);
    private final Scalar range = new Scalar(0,0,0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, HsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(HsvMat,range(redHSV, range, false),range(redHSV, range, true),RedMask);
        Core.inRange(HsvMat,range(blueHSV, range, false),range(redHSV, range, true),BlueMask);
        Core.inRange(HsvMat,range(yellowHSV, range, false),range(redHSV, range, true),YellowMask);

        Imgproc.findContours(RedMask, RedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(BlueMask, BlueContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(YellowMask, YellowContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Mat matCanvas = new Mat(onscreenHeight, onscreenWidth, CvType.CV_8UC3);
        matCanvas.setTo(new Scalar(0, 0, 0));

        Imgproc.drawContours(matCanvas, RedContours, -1, new Scalar(0,0,255), 5;
        Imgproc.drawContours(matCanvas, BlueContours, -1, new Scalar(255,0,0), 5);
        Imgproc.drawContours(matCanvas, YellowContours, -1, new Scalar(0,240,240), 5);
    }

    // helper function for ranges because i'm a lazy ass
    private Scalar range(Scalar input, Scalar range, boolean retUpper) {
        Scalar ret = new Scalar(0,0,0);
        if (retUpper) {
            ret = new Scalar (
                (input.val[0] + range.val[0])%255,
                (input.val[1] + range.val[1])%255,
                (input.val[2] + range.val[2])%255
                );
        } else {
            ret = new Scalar (
                (input.val[0] - range.val[0])%255,
                (input.val[1] - range.val[1])%255,
                (input.val[2] - range.val[2])%255
                );
        }
        return ret;
    }
}
