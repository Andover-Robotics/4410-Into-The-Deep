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
    public static double manualSpeed = 0.7; // need to tune

    public static double target = 0, tolerance = 5;
    private final double ticksPerDegree = (1993.6 * 2.8) / 360.0; //1993.6 is motor tpr + 1:2.8 ratio
    private final double startingAngleOffsetDegrees = 177; //offset from rest position to horizontal front
    private boolean goingDown, IK;

    public static double maxVelo = 30000, maxAccel = 20000;
    private double profilerTarget;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(maxVelo, maxAccel);


    public double targetX, targetZ, slidesTarget;

    public double power, manualPower;

    public int slidesCycler;

    // Constants for gravity compensation
    public static double STATIC_FF = 0.21; // main ff constant of the non-extending part
    public static double EXTENSION_FF = 0.00012; // main ff constant of the extending part
    public static double GRAVITY = 9.81; // gravity :)
    public static double ARM_LENGTH = 0.3; // m, length of the non extendo pivoting arm
    public static double EXTENSION_OFFSET = 0.05; // Extension starts from 0.15m

    public double inches2mm = 25.4;

    public boolean testing = false;
    //BTW angle of 0 degrees is front horizontal - not reachable physically

    // Heights for positions millimeters higher than pivot point
    public double highBucketHeight = 41 * inches2mm,
            lowBucketHeight = 24 * inches2mm,
            highChamberHeight = 34 * inches2mm,
            lowChamberHeight = 20 * inches2mm,
            frontIntakeHeight = 1 * inches2mm,
            wallIntakeHeight = 12 * inches2mm;

    // distances forward from pivot for positions
    public double bucketX = -14 * inches2mm,
            chamberX = 14 * inches2mm,
            frontIntakeX = 11 * inches2mm,
            rearIntakeX = -11 * inches2mm,
            wallIntakeX = -11 * inches2mm;

    // static positions
    public double storageX = 6 * inches2mm, storageZ = 4 * inches2mm;

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

    public void setTesting(boolean testing) {
        this.testing = testing;
    }

    public void runIK (double joystickValue) {
        if (Math.abs(joystickValue) > 0.05) {
            targetZ += joystickValue * 0.2; //increments joystick
            targetX = (1 / Math.tan(getPivotAngleRadians())) * targetZ; //updates the x value so that slides can adjust in the periodic cycler
        }
    }

    public void periodic() {
        controller.setPID(p, i, d);
        int pivotPos = getPosition();
        double ff = calculateFeedForward();  // Calculate gravity compensation
        double dt = opMode.time - profile_init_time;

        if (!profiler.isOver()) {
            profilerTarget = profiler.motion_profile_pos(dt);
            double pid = controller.calculate(pivotPos, profilerTarget);  // PID calculation
            power = pid;
        } else {
            if (profiler.isDone()) {
                resetProfiler();
            }
            // Check if manual control is active
            if (manualPower != 0) {
                // Use joystick input directly
                power = manualPower;
            } else {
                double pid = controller.calculate(pivotPos, profilerTarget);  // hold position
                power = pid;
            }
        }

        // Add feedforward
        power += ff;
        // Set motor power with limits between -1 and 1
        power = Math.max(Math.min(power, 1.0), -1.0);

        if (testing) { //TODO remove for master code
            pivotMotor.set(power);
        } else {
            pivotMotor.set(0);
        }
        //TODO below code is commented out just to make testing the pivot easier - comment it in to use the slides with IK
//        slidesTarget = Math.sqrt(targetX*targetX + targetZ*targetZ);
//        slides.runToMM(slidesTarget);

        slides.periodic(getPivotTargetAngleRadians());
        //arm.periodic(getPivotTargetAngleDegrees());
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

    public void highBucket() {
        targetZ = highBucketHeight;
        targetX = bucketX;
    }

    public void lowBucket() {
        targetZ = lowBucketHeight;
        targetX = bucketX;
    }

    public void highChamber() {
        targetZ = highChamberHeight;
        targetX = chamberX;
    }

    public void lowChamber() {
        targetZ = lowChamberHeight;
        targetX = chamberX;
    }

    public void rearIntake() {
        targetZ = frontIntakeHeight;
        targetX = rearIntakeX;
    }

    public void frontIntake() {
        targetZ = frontIntakeHeight;
        targetX = frontIntakeX;
    }

    public void storage() {
        targetZ = storageZ;
        targetX = storageX;
    }

    public void wallIntake() {
        targetZ = wallIntakeHeight;
        targetX = wallIntakeX;
    }

    public void changeHeight(double inches) {
        targetZ += inches * inches2mm;
    }

    public void frontIntakeStorage() {
        targetZ = frontIntakeHeight + 3 * inches2mm;
        targetX = 8 * inches2mm;
    }

    public double calculateDegXZ(double x, double z) {
        return Math.toDegrees(Math.atan2(z, x));
    }

    public void runToTicks(double pos) {
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(tolerance);
        goingDown = pos > target;
        target = pos;
    }

    public void runToDeg(double angle) {
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        int pos = degreestoTicks(angle);

        controller.setTolerance(tolerance);
        goingDown = pos > target;
        target = pos;
    }

    public void runManual(double manual) {
        if (manual > 0.1 || manual < -0.1) {
            manualPower = manual * manualSpeed;
        } else {
            manualPower = 0;
        }
    }

    public int getPosition() {
        // Get the current motor encoder position
        return pivotMotor.getCurrentPosition();
    }

    public double getPivotAngleDegrees() {
        // Convert the current position in encoder ticks to degrees
        return startingAngleOffsetDegrees - getPosition() / ticksPerDegree;
    }

    public double getPivotAngleRadians() {
        // Convert the current position in encoder ticks to degrees
        return Math.toRadians((startingAngleOffsetDegrees - getPosition() / ticksPerDegree));
    }

    public double getPivotTargetAngleDegrees() {
        return startingAngleOffsetDegrees - target / ticksPerDegree;
    }

    public double getPivotTargetAngleRadians() {
        return Math.toRadians((startingAngleOffsetDegrees - target / ticksPerDegree));
    }

    public int degreestoTicks(double degrees) {
        return (int) (Math.round((startingAngleOffsetDegrees - degrees) * ticksPerDegree));
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
        slides.resetEncoder();
    }

}
