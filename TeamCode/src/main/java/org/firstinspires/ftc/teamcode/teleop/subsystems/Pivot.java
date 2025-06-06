package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class Pivot {

    private PIDController controller;
    public final MotorEx pivotMotor;
    public Slides slides;
    public Arm arm;

    public Bot.BotState state;

    private final OpMode opMode;


    public static double p = 0.0054, i = 0, d = 0.00025, f = 0; // NEED TO TUNE F FIRST WITH FULLY IN ARM - acts as static f constant for gravity

    public static double target = 0, tolerance = 5, powerUp = -0.13;
    private final double ticksPerDegree = (1993.6 * 2.8) / 360.0; //1993.6 is motor tpr + 1:2.8 ratio
    public static final double startingAngleOffsetDegrees = 90; //offset from rest position to horizontal front
    private boolean goingDown, limits;

    public static double maxVelo = 2000, maxAccel = 60000;
    private double profilerTarget;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(maxVelo, maxAccel);


    int position;


    public double targetX, targetZ, slidesTarget, pivotIKTargetDegrees;
    public boolean pivotOff = false;

    public double power, manualPower, manualPowerUp;

    // Constants for gravity compensation
    public static double STATIC_FF = 0.21; // main ff constant of the non-extending part
    public static double EXTENSION_FF = 0.000135; // main ff constant of the extending part
    public static double GRAVITY = 9.81; // gravity :) jerry from lazer says that why do u need this lmao
    public static double ARM_LENGTH = 0.3; // m, length of the non extendo pivoting arm
    public static double EXTENSION_OFFSET = 0.05; // Extension starts from 0.15m

    public static double inches2mm = 25.4;

    public double xMin = -10 * inches2mm, xMax = 21.5 * inches2mm; //expansion limits

    public boolean manualIK;
    //BTW angle of 0 degrees is front horizontal - not reachable physically

    public static double slidesClippingHeightDelta = 8;

    // Heights for positions millimeters higher than pivot point
    public static double highBucketHeight = 36 * inches2mm,
            lowBucketHeight = 21 * inches2mm,
            highChamberHeight = 15.2 * inches2mm,
            lowChamberHeight = 3 * inches2mm,
            frontIntakeHeight = 3.75 * inches2mm,
            wallIntakeHeight = 3.2 * inches2mm,

            frontWallIntakeHeight = 5.25 * inches2mm,
            rearSlidesClippingHeight = 19.25 * inches2mm,

    slidesHighChamberHeight = 20.5 * inches2mm,

    frontAutoIntakeHeight = 5.5 * inches2mm,
    subAutoIntakeHeight = 12 * inches2mm, //12
    pushIntakeHeight = 2.1 * inches2mm,

    prel2ClimbHeight = 22 * inches2mm,
            midl2ClimbHeight = 19 * inches2mm,
            postl2ClimbHeight = 3.5 * inches2mm,

    climbTransferHeight = 15 * inches2mm,
            prel3ClimbHeight = 30.5 * inches2mm,
            midl3ClimbHeight = 22 * inches2mm,
            tiltedl3ClimbHeight = 9.5 * inches2mm,
            backTiltedl3ClimbHeight = 8 * inches2mm,
            postl3ClimbHeight = 5 * inches2mm;

    // distances forward from pivot for positions
    public static double bucketX = -4.75 * inches2mm,
            chamberX = 19.4 * inches2mm,
            highChamberTransferX = 2 * inches2mm,
            frontIntakeX = 16.5 * inches2mm,
            rearIntakeX = -9.5 * inches2mm,
            teleopWallIntakeX = -7.1 * inches2mm,
            autoWallIntakeX = -Math.sqrt(Math.pow(11.86, 2) - Math.pow((wallIntakeHeight/inches2mm), 2)) * inches2mm,
            autoMoreWallIntakeX = autoWallIntakeX = -Math.sqrt(Math.pow(11.86, 2) - Math.pow((wallIntakeHeight/inches2mm), 2)) * inches2mm,

            frontWallIntakeX = 8 * inches2mm,
            rearSlidesClippingX = -5.25 * inches2mm,

    slidesHighChamberX = 4.5 * inches2mm,
    autoSlidesHighChamberX = 7.5 * inches2mm,

    frontAutoDiagIntakeX = 15.9 * inches2mm,
    frontAutoIntakeX = 13 * inches2mm,
    subAutoIntakeX = 14 * inches2mm, //13
    pushIntakeX = 19 * inches2mm,

    prel2ClimbX = 18 * inches2mm,
            midl2ClimbX = 16 * inches2mm,
            postl2ClimbX = 5 * inches2mm,

    climbTransferX = 1.6 * inches2mm,
            prel3ClimbX = 0.1 * inches2mm,
            midl3ClimbX = 5.1 * inches2mm,
            tiltedl3ClimbX = -9.5 * inches2mm,
            backTiltedl3ClimbX = 6 * inches2mm,
            postl3ClimbX = 0.5 * inches2mm;



    //STORAGE
    public double storageX = 4.5 * inches2mm, storageZ = 6 * inches2mm;

    //SUB PARK
    public double subParkX = 9 * inches2mm, subParkZ = 15 * inches2mm;

    //TODO: HYPOTENUSE FOR X AND Y SHOULD NEVER BE LESS THAN 11.86 INCHES - WILL SCREW UP INVERSE KINEMATICS and is not accurate to reality

    public Pivot(OpMode opMode) {
        pivotMotor = new MotorEx(opMode.hardwareMap, "pivotMotor", Motor.GoBILDA.RPM_84);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);

        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setInverted(false);

        // Initialize the slides object
        slides = new Slides(opMode);
        arm = new Arm(opMode);

        this.opMode = opMode;
    }


    public void runManualIK(double joystick) {
        if (Math.abs(joystick) > 0.1) {
            if (joystick > 0 && targetX < xMax && targetX > xMin) {
                slides.runManual(Math.pow(joystick, 2));
            } else if (joystick < 0) {
                slides.runManual(-Math.pow(joystick, 2));
            } else {
                slides.runManual(0);
            }
            adjustTargetX();
            updatePivotManualIK();
        } else {
            slides.runManual(0);
        }
    }

    public void adjustTargetX() {
        if (targetX < -50) { //these units are in MM
            targetX = -Math.sqrt(Math.pow(slides.getIKmmPosition(), 2) - Math.pow(targetZ, 2)); //updates the x value so that pivot can adjust
            if (Double.isNaN(targetX)) targetX = -50; //stops overflow for ik from front to back
        } else if (targetX > 100) {
            targetX = Math.sqrt(Math.pow(slides.getIKmmPosition(), 2) - Math.pow(targetZ, 2)); //updates the x value so that pivot can adjust
            if (Double.isNaN(targetX)) targetX = 100; //stops overflow for ik from front to back
        }
    }

    public void updatePivotManualIK() {
        pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        manualRunToDeg(pivotIKTargetDegrees);
    }

    public void runIKSlides() { //runTo for slides (NOT MANUAL IK)
        slidesTarget = Math.sqrt(targetX * targetX + targetZ * targetZ);
        slides.runToIKMM(slidesTarget);
    }

    public void runToIKPosition() { //runTo - NOT MANUAL IK
        pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        runToDeg(pivotIKTargetDegrees);
        runIKSlides();
    }

    public void runPivotToIKPosition() { //runTo - NOT MANUAL IK
        pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        runToDeg(pivotIKTargetDegrees);
    }

    public void runSlidesToIKPosition() { //runTo - NOT MANUAL IK
        runIKSlides();
    }

    public void periodic(int position, boolean wallIntake) {
        controller.setPID(p, i, d);
        this.position = -position;
        int pivotPos = getPosition();
        double ff = calculateFeedForward();  // Calculate gravity compensation
        double dt = opMode.time - profile_init_time;
        if (!manualIK) {
            if (!profiler.isOver()) {
                double proTar = profiler.motion_profile_pos(dt);
                if (!Double.isNaN(proTar)) {
                    profilerTarget = proTar;
                }
                double pid = controller.calculate(pivotPos, profilerTarget);  // PID calculation
                power = pid;
            } else {
                if (profiler.isOver()) {
                    resetProfiler();
                }
                double pid = controller.calculate(pivotPos);  // hold position
                power = pid;
            }
        } else {
            double pid = controller.calculate(pivotPos, target);  // move PID manually
            power = pid;
            if (!goingDown) {
                manualPowerUp = Math.cos(getPivotTargetAngleRadians()) * powerUp;
            } else { manualPowerUp = 0; }
            power += manualPowerUp;
        }

        // Add feedforward
        power += ff;
        // Set motor power with limits between -1 and 1
        power = Math.max(Math.min(power, 1.0), -1.0);

//        if (pivotOff) {
//            power = 0;
//            pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        } else {
//            pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        }

        pivotMotor.set(power);

        slides.periodic(getPivotAngleRadians(), wallIntake);
        arm.periodic(getPivotAngleDegrees()); // feeds in the pivot angle so the arm can go to an absolute angle
    }

    public double calculateFeedForward() {
        //convert from encoder ticks to degrees
        double angleInRadians = getPivotAngleRadians();

        // Effective length of the extension (changes dynamically based on extension)
        double effectiveExtensionLength = EXTENSION_OFFSET + slides.getmmPosition(); // calculates ext length

        // torque from the static (non extending) arm mass (assumed as a uniform rod, the mass is centered at length/3 due to motors being closer to pivot point)
        double armGravityTorque = STATIC_FF * GRAVITY * (ARM_LENGTH / 3.0);

        // torque from the extending arm
        double extensionGravityTorque = EXTENSION_FF * GRAVITY * (effectiveExtensionLength / 2.0);

        // total ff gravity compensation power
        double totalFeedforwardPower;
        totalFeedforwardPower = (armGravityTorque + extensionGravityTorque) * -(Math.cos(angleInRadians));

        return totalFeedforwardPower;
    }

    public void runToDeg(double angle) {
        int pos = degreestoTicks(angle);
        runTo(pos);
    }

    public void runTo(double pos) {
        if (!Double.isNaN(pos)) {
            manualIK = false;
            pivotMotor.setRunMode(Motor.RunMode.RawPower);
            pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            controller.setTolerance(tolerance);
            resetProfiler();
            profiler.init_new_profile(getPosition(), pos);
            profile_init_time = opMode.time;

            target = pos;

            goingDown = Math.sin(getPivotAngleRadians()) > Math.sin(getPivotTargetAngleRadians());
        }
    }

    public void manualRunTo(double pos) {
        if (target != pos && !Double.isNaN(pos)) {
            pivotMotor.setRunMode(Motor.RunMode.RawPower);
            pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            controller.setTolerance(tolerance);

            target = pos;
            manualIK = true;

            goingDown = Math.sin(getPivotAngleRadians()) > Math.sin(getPivotTargetAngleRadians());
        }
    }

    public void manualRunToDeg(double angle) {
        manualRunTo(degreestoTicks(angle));
    }

    public void rearSlidesChamber(boolean pivot, boolean slides) {
        targetZ = rearSlidesClippingHeight;
        targetX = rearSlidesClippingX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void frontWallIntake(boolean pivot, boolean slides) {
        targetZ = frontWallIntakeHeight;
        targetX = frontWallIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void rearSlidesClipDown(boolean pivot, boolean slides) {
        changeXZ(0, -slidesClippingHeightDelta, pivot, slides);
    }

    public void rearSlidesClipCancel(boolean pivot, boolean slides) {
        changeXZ(0, slidesClippingHeightDelta, pivot, slides);
    }

    public void slidesHighChamber(boolean pivot, boolean slides) {
        targetZ = slidesHighChamberHeight;
        targetX = slidesHighChamberX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void autoSlidesHighChamber(boolean pivot, boolean slides) {
        targetZ = slidesHighChamberHeight;
        targetX = autoSlidesHighChamberX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void slidesClip() {
        changeZ(-slidesClippingHeightDelta);
    }

    public void slidesUnClip() {
        changeZ(slidesClippingHeightDelta);
    }

    public void goToResetPosition() {
        targetX = 0.001; //millimeters
        targetZ = 20; //millimeters
        runPivotToIKPosition();
        runSlidesToIKPosition();
    }

    public void pushIntake(boolean pivot, boolean slides) {
        targetZ = pushIntakeHeight;
        targetX = pushIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void subAutoIntake(boolean pivot, boolean slides) {
        targetZ = subAutoIntakeHeight;
        targetX = subAutoIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void subPark(boolean pivot, boolean slides) {
        targetZ = subParkZ;
        targetX = subParkX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void highBucket(boolean pivot, boolean slides) {
        targetZ = highBucketHeight;
        targetX = bucketX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void lowBucket(boolean pivot, boolean slides) {
        targetZ = lowBucketHeight;
        targetX = bucketX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void highChamber(boolean pivot, boolean slides) {
        targetZ = highChamberHeight;
        targetX = chamberX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void highChamberTransfer(boolean pivot, boolean slides) {
        targetZ = highChamberHeight - (0.9 * inches2mm);
        targetX = highChamberTransferX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void lowChamber(boolean pivot, boolean slides) {
        targetZ = lowChamberHeight;
        targetX = chamberX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void frontIntake(boolean pivot, boolean slides) {
        targetZ = frontIntakeHeight;
        targetX = frontIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void frontAutoIntake(boolean pivot, boolean slides) {
        targetZ = frontAutoIntakeHeight;
        targetX = frontAutoIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void frontAutoDiagIntake(boolean pivot, boolean slides) {
        targetZ = frontAutoIntakeHeight;
        targetX = frontAutoDiagIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void storage(boolean pivot, boolean slides) {
        targetZ = storageZ;
        targetX = storageX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void teleopWallIntake(boolean pivot, boolean slides) {
        targetZ = wallIntakeHeight;
        targetX = teleopWallIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void autoWallIntake(boolean pivot, boolean slides) {
        targetZ = wallIntakeHeight;
        targetX = autoWallIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void autoMoreWallIntake(boolean pivot, boolean slides) {
        targetZ = wallIntakeHeight;
        targetX = autoMoreWallIntakeX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void prel2Climb(boolean pivot, boolean slides) {
        targetZ = prel2ClimbHeight;
        targetX = prel2ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void midl2Climb(boolean pivot, boolean slides) {
        targetZ = midl2ClimbHeight;
        targetX = midl2ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void postl2Climb(boolean pivot, boolean slides) {
        targetZ = postl2ClimbHeight;
        targetX = postl2ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void climbTransfer(boolean pivot, boolean slides) {
        targetZ = climbTransferHeight;
        targetX = climbTransferX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void prel3Climb(boolean pivot, boolean slides) {
        targetZ = prel3ClimbHeight;
        targetX = prel3ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void midl3Climb(boolean pivot, boolean slides) {
        targetZ = midl3ClimbHeight;
        targetX = midl3ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void tiltedl3Climb(boolean pivot, boolean slides) {
        targetZ = tiltedl3ClimbHeight;
        targetX = tiltedl3ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void backTiltedl3Climb(boolean pivot, boolean slides) {
        targetZ = backTiltedl3ClimbHeight;
        targetX = backTiltedl3ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void postl3Climb(boolean pivot, boolean slides) {
        targetZ = postl3ClimbHeight;
        targetX = postl3ClimbX;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void changeXZ(double x, double z, boolean pivot, boolean slides) {
        targetX += x * inches2mm;
        targetZ += z * inches2mm;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public void changeXZ(double x, double z) {
        targetX += x * inches2mm;
        targetZ += z * inches2mm;
        runToIKPosition();
    }

    public void changeZ(double inches) {
        targetZ += inches * inches2mm;
        runToIKPosition();
    }

    public void changeX(double inches) {
        targetX += inches * inches2mm;
        runToIKPosition();
    }

    public void frontIntakeStorage(boolean pivot, boolean slides) {
        targetZ = frontIntakeHeight + 3 * inches2mm;
        targetX = 8 * inches2mm;
        if (pivot) runPivotToIKPosition();
        if (slides) runSlidesToIKPosition();
    }

    public double calculateDegXZ(double x, double z) {
        return Math.toDegrees(Math.atan2(z, x));
    }

    public int getPosition() {
        // Get the current motor encoder position
        return position;
    }

    public double getPivotAngleDegrees() {
        // Convert the current position in encoder ticks to degrees
        return startingAngleOffsetDegrees - getPosition() / ticksPerDegree;
    }

    public double getPivotAngleRadians() {
        // Convert the current position in encoder ticks to degrees
        return Math.toRadians(getPivotAngleDegrees());
    }

    public double getPivotTargetAngleDegrees() {
        return startingAngleOffsetDegrees - target / ticksPerDegree;
    }

    public double getPivotTargetAngleRadians() {
        return Math.toRadians(getPivotTargetAngleDegrees());
    }

    public int degreestoTicks(double degrees) {
        return (int) (Math.round((startingAngleOffsetDegrees - degrees) * ticksPerDegree));
    }

    public double getTarget() {
        return target;
    }

    public double getProfilerTarget() {
        return controller.getSetPoint();
    }

    public double getCurrent() {
        return pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(maxVelo, maxAccel);
    }

    public void resetEncoder() {
        pivotMotor.resetEncoder();
        slides.resetEncoders();
    }

    public double getDTerm() {
        return power - controller.getPositionError() * p - calculateFeedForward();
    }

    public double getPTerm() {
        return controller.getPositionError() * p;
    }

}
