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
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.Objects;

@Config
public class SampleDetectionPipeline
{

    public int height = 360, width = 640;
    public static int minArea = 7500, maxArea = 19500;
    public int rounds;
    public static int strafePixelsPerInch = -42;
    public static int slidesPixelsPerInch = 32;
    public static int slidesRescaledMax = 270;
    public static int strafeRescaledMax = 570;

    public static double decel = -25;
    public static double accel = 55;

    public static double strafeA = -0.00000156672, strafeB = -0.0199432, strafeC = 0/*334958 + 0.1*/, strafeMultiplier = 1.52;
//    public static double slidesA = 0.0193807, slidesB = 1.3, slidesMultiplier = 1.2, strafingSlidesModifier = -0;

//    public static double strafeA = -0.0, strafeB = -0.0195, strafeC = 0.23/*334958 + 0.1*/, strafeMultiplier = 1.5;
    public static double slidesA = 0.02301524, slidesB = 2.2, slidesMultiplier = 1.0, strafingSlidesModifier = -0.0;//-0.1 and 0.02112524 for A

    public static double lowAR = 1.2, highAR = 3.1, lowD = 0.4, highD = 1;

    public static double pastStrafe = 0;
    public double aspectRatio = 0, side1 = 0, side2 = 0, boxArea = 0, blobArea = 0, density = 0;
    private int iterCounter = 0;
    public boolean red = false, blue = false, yellow = false;
    private final ColorBlobLocatorProcessor blueLocator;
    private final ColorBlobLocatorProcessor redLocator;
    private final ColorBlobLocatorProcessor yellowLocator;
    List<ColorBlobLocatorProcessor.Blob> blobs, badBlobs;
    RotatedRect boxFit;
    List<Point> badCenters;
    ColorBlobLocatorProcessor.Blob bigBlob;
    Point center;
    boolean goodBlob = false;

    VisionPortal.Builder portalBuilder;

    VisionPortal portal;

    private static double angle, x, y;

    public static final ColorRange BLUE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16,   0, 155),
            new Scalar(255, 127, 255)
    );

    public static final ColorRange RED = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 176,  0),
            new Scalar(255, 255, 132)
    );

    public static final ColorRange YELLOW = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 110, 128,   0),
            new Scalar(255, 170, 120)
    );

    public SampleDetectionPipeline(boolean red, boolean blue, boolean yellow, HardwareMap map) {
        this.red = red;
        this.blue = blue;
        this.yellow = yellow;

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setBlurSize(2)
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setBlurSize(2)
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setBlurSize(2)
                .build();

        portalBuilder = new VisionPortal.Builder();

        portalBuilder.addProcessor(blueLocator);
        portalBuilder.addProcessor(yellowLocator);
        portalBuilder.addProcessor(redLocator);
        portalBuilder.setCameraResolution(new Size(width, height));
        portalBuilder.setCamera(map.get(WebcamName.class, "Webcam 1"));
        portalBuilder.setLiveViewContainerId(0);
        portal = portalBuilder.build();

        x = 0;
        y = 0;
    }

    public void close() {
        portal.close();
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
//            badBlobs = redLocator.getBlobs();
        } else if (red) {
            blobs = redLocator.getBlobs();
//            badBlobs = blueLocator.getBlobs();
        }

        if (yellow) {
            if (red || blue) {
                blobs.addAll(yellowLocator.getBlobs());
            } else {
                blobs = yellowLocator.getBlobs();
            }
        }
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

//        ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, badBlobs);
//        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, badBlobs);

        if (!blobs.isEmpty()) {
            for (ColorBlobLocatorProcessor.Blob myBlob : blobs) {
                bigBlob = myBlob;
                boxFit = bigBlob.getBoxFit();

                center = boxFit.center;
                x = (double) center.x - (double) width / 2;
                y = (double) center.y - (double) height / 2;
                angle = ((int) ((boxFit.angle + 90) % 180));
                Point[] myBoxCorners = new Point[4];
                boxFit.points(myBoxCorners);
                if ((Math.sqrt(Math.pow((myBoxCorners[0].x) - (myBoxCorners[1].x), 2) + Math.pow((myBoxCorners[0].y) - (myBoxCorners[1].y), 2))) >
                        (Math.sqrt(Math.pow((myBoxCorners[1].x) - (myBoxCorners[2].x), 2) + Math.pow((myBoxCorners[1].y) - (myBoxCorners[2].y), 2)))) {
                    angle = ((int) ((boxFit.angle) % 180));
                    side1 = (Math.sqrt(Math.pow((myBoxCorners[0].x) - (myBoxCorners[1].x), 2) + Math.pow((myBoxCorners[0].y) - (myBoxCorners[1].y), 2)));
                    side2 = (Math.sqrt(Math.pow((myBoxCorners[1].x) - (myBoxCorners[2].x), 2) + Math.pow((myBoxCorners[1].y) - (myBoxCorners[2].y), 2)));
                } else {
                    angle = (int) ((boxFit.angle + 90) % 180);
                    side1 = (Math.sqrt(Math.pow((myBoxCorners[1].x) - (myBoxCorners[2].x), 2) + Math.pow((myBoxCorners[1].y) - (myBoxCorners[2].y), 2)));
                    side2 = (Math.sqrt(Math.pow((myBoxCorners[0].x) - (myBoxCorners[1].x), 2) + Math.pow((myBoxCorners[0].y) - (myBoxCorners[1].y), 2)));
                }
                aspectRatio = side1 / side2;
                blobArea = bigBlob.getContourArea();
                boxArea = side1 * side2;
                try {
                    density = blobArea/boxArea;
                } catch (Exception e) {
                    density = 0;
                }
                if (aspectRatio > lowAR && aspectRatio < highAR && density > lowD && density < highD) {
                    break;
                } else {
                    x = 2000;
                    y = 2000;
                    angle = -1;
                    blobArea = 0;
                    boxArea = 0;
                    density = 0;
                    aspectRatio = 0;
                }
            }
        } else {
            x = 2000;
            y = 2000;
            angle = -1;
            blobArea = 0;
            boxArea = 0;
            density = 0;
            aspectRatio = 0;
        }
    }

    public double getAngle() {
        return angle;
    }

    public double getAspectRatio() {
        return aspectRatio;
    }

    public double getX() {
        if (x != 2000) {
//            double value = ((x + 320)/640.0 * strafeRescaledMax - (strafeRescaledMax-320))/strafePixelsPerInch;
//            value += Math.max(0, ((value - 3.5) * 0.5874 - 0.5));
            double value = strafeA * x * x + strafeB * x + strafeC;
            return value; //NEEDS TO return in inches
        } else {
            return 0;
        }
    }

    public double getDriveX() {
        return getX() * strafeMultiplier;
    }

    public double getAX() {
        return strafeA * x * x * strafeMultiplier;
    }

    public double getXPixels() {
        return x;
    }

    public double getYPixels() {
        return y;
    }

    public double getY() {
        if (y != 2000) {
            double value = slidesA * y + slidesB;
            return value; //NEEDS TO return in inches
        } else {
            return 0;
        }
//            return ((y + 180)/360.0 * slidesRescaledMax - (slidesRescaledMax-180))/slidesPixelsPerInch; //NEEDS To return in inches OLD VERSIOn
    }

    public double getSlidesY() {
        return getY() * slidesMultiplier + getX() * strafingSlidesModifier;
    }

    public double getArea() {
        return blobArea;
    }

    public double getDensity() {
        return density;
    }

}