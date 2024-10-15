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
    public static double manualSpeed = 0.3; // need to tune

    public static double target = 0, tolerance = 5, powerUp = -0.13;
    private final double ticksPerDegree = (1993.6 * 2.8) / 360.0; //1993.6 is motor tpr + 1:2.8 ratio
    private final double startingAngleOffsetDegrees = 177; //offset from rest position to horizontal front
    private boolean goingDown, IK;

    public static double maxVelo = 1000, maxAccel = 60000;
    private double profilerTarget;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(maxVelo, maxAccel);


    public double targetX, targetZ, slidesTarget, pivotIKTargetDegrees;

    public double power, manualPower, manualPowerUp;

    public int slidesCycler;

    // Constants for gravity compensation
    public static double STATIC_FF = 0.21; // main ff constant of the non-extending part
    public static double EXTENSION_FF = 0.00012; // main ff constant of the extending part
    public static double GRAVITY = 9.81; // gravity :)
    public static double ARM_LENGTH = 0.3; // m, length of the non extendo pivoting arm
    public static double EXTENSION_OFFSET = 0.05; // Extension starts from 0.15m

    public double inches2mm = 25.4;

    public boolean testing = false, manualIK;
    //BTW angle of 0 degrees is front horizontal - not reachable physically

    // Heights for positions millimeters higher than pivot point
    public double highBucketHeight = 41 * inches2mm,
            lowBucketHeight = 24 * inches2mm,
            highChamberHeight = 34 * inches2mm,
            lowChamberHeight = 20 * inches2mm,
            frontIntakeHeight = 6.5 * inches2mm,
            wallIntakeHeight = 8 * inches2mm;

    // distances forward from pivot for positions
    public double bucketX = -2.5 * inches2mm,
            chamberX = 8 * inches2mm,
            frontIntakeX = Math.sqrt(Math.pow(11.86, 2) - Math.pow((frontIntakeHeight/inches2mm), 2)) * inches2mm * inches2mm,
            rearIntakeX = -Math.sqrt(Math.pow(11.86, 2) - Math.pow((frontIntakeHeight/inches2mm), 2)) * inches2mm,
            wallIntakeX = Math.sqrt(Math.pow(11.86, 2) - Math.pow((wallIntakeHeight/inches2mm), 2)) * inches2mm;
    //STORAGE
    public double storageX = 6 * inches2mm, storageZ = Math.sqrt(Math.pow(11.86, 2) - Math.pow((storageX/inches2mm), 2)) * inches2mm;

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

    public void setTesting(boolean testing) {
        this.testing = testing;
    }

    public void runManualIK(double joystick) {
        if (Math.abs(joystick) > 0.05) {
            slides.runManual(joystick);
            adjustTargetX();
            updatePivotManualIK();
        }
    }

    public void adjustTargetX() {
        if (targetX < 0) {
            targetX = -Math.sqrt(Math.pow(slides.getIKmmPosition(), 2) - Math.pow(targetZ, 2)); //updates the x value so that pivot can adjust
        } else if (targetX > 0) {
            targetX = Math.sqrt(Math.pow(slides.getIKmmPosition(), 2) - Math.pow(targetZ, 2)); //updates the x value so that pivot can adjust
        }
    }

    public void updatePivotManualIK() {
        pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        manualRunToDeg(pivotIKTargetDegrees);
    }

    public void runIKSlides() {
        slidesTarget = Math.sqrt(targetX * targetX + targetZ * targetZ);
        slides.runToIKMM(slidesTarget);
    }

    public void runToIKPosition() {
        double pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        runToDeg(pivotIKTargetDegrees);
        runIKSlides();
    }

    public void runPivotToIKPosition() {
        double pivotIKTargetDegrees = calculateDegXZ(targetX, targetZ);
        runToDeg(pivotIKTargetDegrees);
    }

    public void runSlidesToIKPosition() {
        runIKSlides();
    }

    public void periodic() {
        controller.setPID(p, i, d);
        int pivotPos = getPosition();
        double ff = calculateFeedForward();  // Calculate gravity compensation
        double dt = opMode.time - profile_init_time;
        if (!manualIK) {
            if (!profiler.isOver()) {
                profilerTarget = profiler.motion_profile_pos(dt);
                double pid = controller.calculate(pivotPos, profilerTarget);  // PID calculation
                power = pid;
            } else {
                if (profiler.isDone()) {
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

        if (testing) { //TODO remove for master code
            pivotMotor.set(power);
        } else {
            pivotMotor.set(0);
        }

        slides.periodic(getPivotTargetAngleRadians());
        //arm.periodic(getPivotTargetAngleDegrees()); // feeds in the pivot angle so the arm can go to an absolute angle
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
        if (target != pos && !Double.isNaN(pos)) {
            pivotMotor.setRunMode(Motor.RunMode.RawPower);
            pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            controller.setTolerance(tolerance);
            resetProfiler();
            profiler.init_new_profile(getPosition(), pos);
            profile_init_time = opMode.time;

            target = pos;
            manualIK = false;

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

//    public void runManual(double manual) {
//        if (manual > 0.1 || manual < -0.1) {
//            manualPower = manual * manualSpeed;
//        } else {
//            manualPower = 0;
//        }
//        this.manualIK = false; //turning manualIK PID off
//    }

    public void highBucket() {
        targetZ = highBucketHeight;
        targetX = bucketX;
        runToIKPosition();
    }

    public void lowBucket() {
        targetZ = lowBucketHeight;
        targetX = bucketX;
        runToIKPosition();
    }

    public void highChamber() {
        targetZ = highChamberHeight;
        targetX = chamberX;
        runToIKPosition();
    }

    public void lowChamber() {
        targetZ = lowChamberHeight;
        targetX = chamberX;
        runToIKPosition();
    }

    public void rearIntake() {
        targetZ = frontIntakeHeight;
        targetX = rearIntakeX;
        runToIKPosition();
    }

    public void frontIntake() {
        targetZ = frontIntakeHeight;
        targetX = frontIntakeX;
        runToIKPosition();
    }

    public void storage() {
        targetZ = storageZ;
        targetX = storageX;
        runToIKPosition();
    }

    public void wallIntake() {
        targetZ = wallIntakeHeight;
        targetX = wallIntakeX;
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

    public void frontIntakeStorage() {
        targetZ = frontIntakeHeight + 3 * inches2mm;
        targetX = 8 * inches2mm;
        runToIKPosition();
    }

    public double calculateDegXZ(double x, double z) {
        return Math.toDegrees(Math.atan2(z, x));
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
        slides.resetEncoder();
    }

    public double getDTerm() {
        return power - controller.getPositionError() * p - calculateFeedForward();
    }

    public double getPTerm() {
        return controller.getPositionError() * p;
    }

}
