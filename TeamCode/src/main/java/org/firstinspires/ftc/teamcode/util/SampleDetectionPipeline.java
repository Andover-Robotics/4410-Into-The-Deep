package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.hardware.HardwareBuffer;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

@Config
public class SampleDetectionPipeline
{

    public int height = 360, width = 640, minArea = 800;
    public static int strafePixelsPerInch = 25;
    public static int slidesPixelsPerInch = 32;
    public static int slidesRescaledMax = 270;
    public boolean red = false, blue = false, yellow = false;
    private final ColorBlobLocatorProcessor blueLocator;
    private final ColorBlobLocatorProcessor redLocator;
    private final ColorBlobLocatorProcessor yellowLocator;
    List<ColorBlobLocatorProcessor.Blob> blobs;
    RotatedRect boxFit;
    ColorBlobLocatorProcessor.Blob bigBlob;
    Point center;
    Camera camera;

    private static double angle, x, y;

    public SampleDetectionPipeline(boolean red, boolean blue, boolean yellow, HardwareMap map) {
        this.red = red;
        this.blue = blue;
        this.yellow = yellow;

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        if (blue) portalBuilder.addProcessor(blueLocator);
        if (yellow) portalBuilder.addProcessor(yellowLocator);
        if (red) portalBuilder.addProcessor(redLocator);
        portalBuilder.setCameraResolution(new Size(width, height));
        portalBuilder.setCamera(map.get(WebcamName.class, "Webcam 1"));
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

            angle = ((int) ((boxFit.angle + 90) % 180));
            Point[] myBoxCorners = new Point[4];
            boxFit.points(myBoxCorners);
            if ((Math.sqrt(Math.pow((myBoxCorners[0].x)-(myBoxCorners[1].x), 2) + Math.pow((myBoxCorners[0].y)-(myBoxCorners[1].y), 2))) >
                    (Math.sqrt(Math.pow((myBoxCorners[1].x)-(myBoxCorners[2].x), 2) + Math.pow((myBoxCorners[1].y)-(myBoxCorners[2].y), 2)))) {
                angle = ((int) ((boxFit.angle) % 180));
            } else {
                angle = (int) ((boxFit.angle + 90) % 180);
            }

            center = boxFit.center;
            x = (double) center.x - (double) width/2;
            y = (double) center.y - (double) height/2; //accounts for claw being below camera
        } else {
            angle = -1;
            y = 4;
        }
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return (x)/strafePixelsPerInch; //NEEDS TO return in inches
    }

    public double getY() {
        return ((y + 180)/360.0 * slidesRescaledMax - (slidesRescaledMax-180))/slidesPixelsPerInch; //NEEDS To return in inches
    }

    public int getArea() {
        if (blobs.isEmpty()) {
            return 0;
        } else {
            return bigBlob.getContourArea();
        }
    }
}