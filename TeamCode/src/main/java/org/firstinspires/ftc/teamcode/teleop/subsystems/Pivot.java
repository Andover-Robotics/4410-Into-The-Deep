package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Pivot {

    private PIDController controller;
    public final MotorEx pivotMotor;
    public Slides slides;

    public enum Position {
        HB, //high bucket
        LB, //low bucket
        HC, //high chamber
        LC, //low chamber
        RIN, //rear intake
        FIN, //front intake
        IN, //all the way in
    }

    public Pivot.Position position = Pivot.Position.IN;
    private final OpMode opMode;

    public static double p = 0, i = 0, d = 0, f = 0; // NEED TO TUNE F FIRST WITH FULLY IN ARM - acts as static f constant for gravity
    public static double manualSpeed = 0.7; // need to tune

    public static double target = 0, tolerance = 20;
    private final double ticksPerDegree = 5281.1 / 360.0;
    private boolean goingDown;

    public double power, manualPower;

    // Constants for gravity compensation
    public static final double ARM_MASS = 2; // kg, mass of the non extendo pivoting arm
    public static final double EXTENSION_MASS = 2; // kg, mass of the extending part
    public static final double GRAVITY = 9.81; // gravity :)
    public static final double ARM_LENGTH = 0.4; // m, length of the non extendo pivoting arm
    public static final double EXTENSION_OFFSET = 0.15; // Extension starts from 0.15m

    public Pivot(OpMode opMode) {
        pivotMotor = new MotorEx(opMode.hardwareMap, "pivot", Motor.GoBILDA.RPM_30);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);

        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the slides object
        slides = new Slides(opMode);

        // Set the reference to this Pivot object in the Slides class
        slides.setPivot(this);

        this.opMode = opMode;
    }

    public void runTo(double pos) {
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(tolerance);
        goingDown = pos > target;
        target = pos;
    }

    public void periodic() {
        controller.setPID(p, i, d);
        int pivotPos = getPosition();

        double ff = calculateFeedForward(pivotPos, target);  // Calculate gravity compensation

        // Check if manual control is active
        if (manualPower != 0) {
            // Use joystick input directly, but add feedforward to hold the arm up against gravity
            power = manualPower + ff;
        } else {
            // Use PID + feedforward control
            double pid = controller.calculate(pivotPos, target);  // PID calculation
            power = pid + ff;
        }

        // Set motor power with limits between -1 and 1
        power = Math.max(Math.min(power, 1.0), -1.0);
        pivotMotor.set(power);
    }

    public void runManual(double manual) {
        if (manual > 0.1 || manual < -0.1) {
            manualPower = manual * manualSpeed;
        } else {
            manualPower = 0;
        }
    }

    public double calculateFeedForward(double pos, double setpoint) {
        //convert from encoder ticks to degrees
        double angleInDegrees = pos / ticksPerDegree;
        double angleInRadians = Math.toRadians(angleInDegrees);

        // Effective length of the extension (changes dynamically based on extension)
        double effectiveExtensionLength = EXTENSION_OFFSET + slides.getmmPosition(); // calculates ext length

        // torque from the static (non extending) arm mass (assumed as a uniform rod, the mass is centered at length/3 due to motors being closer to pivot point)
        double armGravityTorque = ARM_MASS * GRAVITY * (ARM_LENGTH / 2.0);

        // torque from the extending arm
        double extensionGravityTorque = EXTENSION_MASS * GRAVITY * (effectiveExtensionLength / 2.0);

        // total ff gravity compensation power
        double totalFeedforwardPower = (armGravityTorque + extensionGravityTorque) * Math.sin(angleInRadians);

        return totalFeedforwardPower;
    }

    public int getPosition() {
        // Get the current motor encoder position
        return pivotMotor.getCurrentPosition();
    }

    public double getPivotAngleDegrees() {
        // Convert the current position in encoder ticks to degrees
        return getPosition() / ticksPerDegree;
    }

    public double getPivotAngleRadians() {
        // Convert the current position in encoder ticks to degrees
        return Math.toRadians((getPosition() / ticksPerDegree));
    }

    public double getCurrent() {
        return pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetEncoder() {
        pivotMotor.resetEncoder();
        slides.resetEncoder();
    }

    public Pivot.Position getState() {
        return position;
    }

}
