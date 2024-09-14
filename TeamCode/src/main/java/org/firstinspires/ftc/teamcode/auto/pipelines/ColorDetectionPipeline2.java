package org.firstinspires.ftc.teamcode.auto.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipeline2 extends OpenCvPipeline {
    // VARIABLES
    Telemetry telemetry;

    // Processing frames
    private final Mat matYCrCb = new Mat();

    // Cb and Cr predetermined Mats
    private final Mat matCbCenter = new Mat();
    private final Mat matCbLeft = new Mat();
    private final Mat matCrCenter = new Mat();
    private final Mat matCrLeft = new Mat();
    private Mat spikeCenter = new Mat();
    private Mat spikeLeft = new Mat();
    private Mat[] mats = new Mat[2];

    // Average Cb and Cr values
    public double avgCenter = 0, avgLeft = 0;
    public double percent_diff = 0;
    public static double minimumAvg = 130;

    // Configurations
    enum SpikeMark{
        LEFT, MIDDLE, RIGHT, NONE
    }
    public SpikeMark spikeMark = SpikeMark.NONE;
    public int alliance = 0; // Alliance: 0 - NONE, 1 - RED, 2 - BLUE
    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    // CONSTRUCTOR
    public ColorDetectionPipeline2(Telemetry telemetry){
        spikeMark = SpikeMark.NONE;
        this.telemetry = telemetry;
    }

    // Execute logic
    public int getSpikeMark() {
//        if (alliance == 1) {
            if(minimumAvg> avgLeft && minimumAvg> avgCenter){
                return 3;
            }else{
                if(avgLeft>avgCenter){
                    return 1;
                }else{
                    return 2;
                }
            }
//        } else {
//            if(avgCenter> avgLeft && percent_diff< 0.025) {
//                return 3;
//            } else {
//                if (avgCenter> avgLeft) {
//                    return 1;
//                } else {
//                    return 2;
//                }
//            }
//        }

    }

    // PROCESSING FRAMES
    public Mat processFrame(Mat input) {
        // Convert from RGB to YCrCb
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Draw rectangles of left and center positions
        Rect rectCenter = new Rect(300, 62, 340, 120);
        Rect rectLeft = new Rect(75, 75, 180, 125);

        // Create mats for each rectangle
        spikeCenter = matYCrCb.submat(rectCenter);
        spikeLeft = matYCrCb.submat(rectLeft);

        // Extract color based off of alliance
        if (alliance == 1) { // Red
            Core.extractChannel(spikeCenter, matCrCenter, 1);
            Core.extractChannel(spikeLeft, matCrLeft, 1);
            mats[0] = matCrLeft;
            mats[1] = matCrCenter;

        } else{ // Blue
            Core.extractChannel(spikeCenter, matCbCenter, 2);
            Core.extractChannel(spikeLeft, matCbLeft, 2);
//            mats = {matCbLeft, matCbCenter};
            mats[0] = matCbLeft;
            mats[1] = matCbCenter;
        }

        // Calculate average

        double[] sums = new double[2];
        int n = 0;
        for(int i = 0; i < 2; i++) { // left and center
            for (int j = 0; j < mats[i].rows(); j++) { // go through rows
                for (int k = 0; k < mats[i].cols(); k++) { // go through columns
                    sums[i] += mats[i].get(j, k)[0];
                    n++;
                }
            }
            sums[i] /= n;
            n=0;
        }
        avgCenter = sums[1];
        avgLeft = sums[0];

        // normalize values
        percent_diff = (avgCenter/(double) Math.max(avgLeft,avgCenter)) - (avgLeft/(double) Math.max(avgLeft,avgCenter));


        // Display rectangles
        // Left turns red if active
        // Center turns red if active
        // Left and Center turn yellow if right
        // any inactive stay green

//        if (getSpikeMark() == 1) {
//            Imgproc.rectangle(input, rectLeft, new Scalar(255, 0, 0), 5);
//        } else {
//            Imgproc.rectangle(input, rectLeft, new Scalar(0, 255, 0), 5);
//        }
//
//        if (getSpikeMark() == 2) {
//            Imgproc.rectangle(input, rectCenter, new Scalar(0, 255, 0), 5);
//        } else {
//            Imgproc.rectangle(input, rectCenter, new Scalar(255, 0, 0), 5);
//        }
//
//        if (getSpikeMark() == 3) {
//            Imgproc.rectangle(input, rectCenter, new Scalar(255, 255, 0), 5);
//            Imgproc.rectangle(input, rectLeft, new Scalar(255, 255, 0), 5);
//        } else {
            Imgproc.rectangle(input, rectCenter, new Scalar(0, 255, 0), 5);
            Imgproc.rectangle(input, rectLeft, new Scalar(0, 255, 0), 5);
            matYCrCb.release();
//            spikeLeft.release();
//            spikeCenter.release();
//            matCbCenter.release();
//            matCbLeft.release();
        // Return img frame
        return input;
    }

    public double getAvgCenter() { return avgCenter; }
    public double getAvgLeft() { return avgLeft; }

    public void setMinAvg(double minimumAvg) { ColorDetectionPipeline2.minimumAvg = minimumAvg; }
    public double getMinAvg() { return minimumAvg; }
}
