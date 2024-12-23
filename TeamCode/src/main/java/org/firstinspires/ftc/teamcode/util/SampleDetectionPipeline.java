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
import java.util.Objects;

@Config
public class SampleDetectionPipeline
{

    public int height = 360, width = 640, minArea = 7000;
    public int rounds;
    public static int strafePixelsPerInch = -39;
    public static int slidesPixelsPerInch = 31;
    public static int slidesRescaledMax = 270;
    public static int strafeRescaledMax = 570;
    public static double pastStrafe = 0;
    public boolean red = false, blue = false, yellow = false;
    private final ColorBlobLocatorProcessor blueLocator;
    private final ColorBlobLocatorProcessor redLocator;
    private final ColorBlobLocatorProcessor yellowLocator;
    List<ColorBlobLocatorProcessor.Blob> blobs;
    RotatedRect boxFit;
    ColorBlobLocatorProcessor.Blob bigBlob;
    Point center;

    private static double angle, x, y;

    public SampleDetectionPipeline(boolean red, boolean blue, boolean yellow, HardwareMap map) {
        this.red = red;
        this.blue = blue;
        this.yellow = yellow;

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.7, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.7, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.7, 1, -1))  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(2)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        portalBuilder.addProcessor(blueLocator);
        portalBuilder.addProcessor(yellowLocator);
        portalBuilder.addProcessor(redLocator);
        portalBuilder.setCameraResolution(new Size(width, height));
        portalBuilder.setCamera(map.get(WebcamName.class, "Webcam 1"));
        VisionPortal portal = portalBuilder.build();

        x = 0;
        y = 0;
    }

    public void setRed(boolean red) {
        this.red = red;
    }

    public void setBlue(boolean blue) {
        this.blue = blue;
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
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, 18000, blobs);
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
        }
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        if (x != 0) {
            double value = ((x + 320)/640.0 * strafeRescaledMax - (strafeRescaledMax-320))/strafePixelsPerInch;
            value += Math.max(0, ((value - 3.5) * 0.5874 - 0.5));
            return value; //NEEDS TO return in inches
        } else {
            return 0;
        }
    }

    public double getY() {
            return ((y + 180)/360.0 * slidesRescaledMax - (slidesRescaledMax-180))/slidesPixelsPerInch; //NEEDS To return in inches
    }

    public int getArea() {
        if (Objects.isNull(blobs)) {
            return 0;
        } else if (blobs.isEmpty()) {
            return 0;
        } else {
            return bigBlob.getContourArea();
        }
    }

}