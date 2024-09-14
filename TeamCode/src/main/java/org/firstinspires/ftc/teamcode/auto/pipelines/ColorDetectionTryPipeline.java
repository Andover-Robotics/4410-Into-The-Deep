package org.firstinspires.ftc.teamcode.auto.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionTryPipeline extends OpenCvPipeline {
    // VARIABLES
    Telemetry telemetry;
    public static int minimumAvg = 50;

    // Processing frames
    private Mat matYCrCb = new Mat();;
    private Mat spikeCenter = new Mat();
    private Mat spikeLeft = new Mat();

    // Cb and Cr predetermined Mats
    private Mat matCbCenter = new Mat();
    private Mat matCbLeft = new Mat();
    private Mat matCrCenter = new Mat();
    private Mat matCrLeft = new Mat();

    // Average Cb and Cr values
    private double avgCenter = 0;
    private double avgLeft = 0;

    // Configurations
    public enum SpikeMark{
        LEFT, MIDDLE, RIGHT, NONE
    }
    public static SpikeMark spikeMark = SpikeMark.NONE;
    public static int alliance = 0; // Alliance: 0 - NONE, 1 - RED, 2 - BLUE
    public void setAlliance(int alliance) {
        this.alliance = alliance;
    }

    // CONSTRUCTOR
    public ColorDetectionTryPipeline(Telemetry telemetry){
        spikeMark = SpikeMark.NONE;
        this.telemetry = telemetry;
    }

    // Execute logic
    public int getSpikeMark() {
        if (getAvgLeft() < minimumAvg && getAvgCenter() < minimumAvg) { // Right, both avgs is too small
            return 3;
        } else if (getAvgCenter() < getAvgLeft()) { // Left, left avg greater than center avg
            return 1;
        } else if (getAvgLeft() < getAvgCenter()) { // Center, center avg greater than left avg
            return 2;
        }
        return 3;
    }

    // PROCESSING FRAMES
    public Mat processFrame(Mat input) {
        // Convert from RGB to YCrCb
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Draw rectangles of left and center positions
        Rect rectCenter = new Rect(600, 50, 680, 240);
        Rect rectLeft = new Rect(100, 0, 325, 400);
        // Display rectangles
        Imgproc.rectangle(input, rectCenter, new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(input, rectLeft, new Scalar(0, 255, 0), 5);

        // Create mats for each rectangle
        spikeCenter = matYCrCb.submat(rectCenter);
        spikeLeft = matYCrCb.submat(rectLeft);

        // Extract color based off of alliance
        if (alliance == 1) { // Blue
            Core.extractChannel(spikeCenter, matCbCenter, 2);
            Core.extractChannel(spikeLeft, matCbLeft, 2);
            // Calculate average
            Scalar meanCenter = Core.mean(matCbCenter);
            Scalar meanLeft = Core.mean(matCbLeft);
            avgCenter = meanCenter.val[0];
            avgLeft = meanLeft.val[0];
        } else if (alliance == 2) { // Red
            Core.extractChannel(spikeCenter, matCrCenter, 2);
            Core.extractChannel(spikeLeft, matCrLeft, 2);
            // Calculate average
            Scalar meanCenter = Core.mean(matCrCenter);
            Scalar meanLeft = Core.mean(matCrLeft);
            avgCenter = meanCenter.val[0];
            avgLeft = meanLeft.val[0];
        }

        // Return img frame
        return input;
    }

    public double getAvgCenter() { return avgCenter; }
    public double getAvgLeft() { return avgLeft; }
}
