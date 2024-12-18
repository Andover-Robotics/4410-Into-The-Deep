package org.firstinspires.ftc.teamcode.cv;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

public class SampleDetectionPipeline
{

    public int height = 360, width = 640, minArea = 2000;
    public boolean red = false, blue = false, yellow = false;
    private final ColorBlobLocatorProcessor blueLocator;
    private final ColorBlobLocatorProcessor redLocator;
    private final ColorBlobLocatorProcessor yellowLocator;
    List<ColorBlobLocatorProcessor.Blob> blobs;
    RotatedRect boxFit;
    ColorBlobLocatorProcessor.Blob bigBlob;
    Point center;

    private static int angle, x, y;

    public SampleDetectionPipeline(boolean red, boolean blue, boolean yellow) {
        this.red = red;
        this.blue = blue;
        this.yellow = yellow;

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(4)                               // Smooth the transitions between different colors in image
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(4)                               // Smooth the transitions between different colors in image
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(4)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        if (blue) portalBuilder.addProcessor(blueLocator);
        if (yellow) portalBuilder.addProcessor(yellowLocator);
        if (red) portalBuilder.addProcessor(redLocator);
        portalBuilder.setCameraResolution(new Size(width, height));
        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        VisionPortal portal = portalBuilder.build();
    }




    public void detect() {
        if (blue) {
            blobs = blueLocator.getBlobs();
        } else if (red) {
            blobs = redLocator.getBlobs();
        }
        if (yellow) {
            if (red || blue) {
                blobs.addAll(yellowLocator.getBlobs());
            } else {
                blobs = yellowLocator.getBlobs();
            }
        }
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, 1000000, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        if (!blobs.isEmpty()) {
            bigBlob = blobs.get(0);
            boxFit = bigBlob.getBoxFit();

            angle = 180 - ((int) (boxFit.angle % 180));

            center = boxFit.center;
            x = (int) center.x - width/2;
            y = (int) center.y - height/2;
        } else {
            angle = -1;
        }
    }

    public int getAngle() {
        return angle;
    }

    public int getX() {
        return x/5; //NEEDS TO return in mm
    }

    public int getY() {
        return y/70; //NEEDS To return in inches
    }
}