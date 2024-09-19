package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
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

    public static double p = 0, i = 0, d = 0;

    public static double target = 0;
    private final double ticksPerDegree = 5281.1 / 360.0;

    public double power;

    // Constants for gravity compensation
    public static final double ARM_MASS = 2; // kg, mass of the non extendo pivoting arm
    public static final double EXTENSION_MASS = 2; // kg, mass of the extending part
    public static final double GRAVITY = 9.81; // NEED TO TUNE THIS FIRST WITH FULLY IN ARM - acts as static f constant for gravity
    public static final double ARM_LENGTH = 0.4; // m, length of the non extendo pivoting arm
    public static final double EXTENSION_OFFSET = 0.15; // Extension starts from 0.15m

    public Pivot(OpMode opMode) {
        pivotMotor = new MotorEx(opMode.hardwareMap, "pivot", Motor.GoBILDA.RPM_30);
        controller = new PIDController(p, i, d);
        this.opMode = opMode;
    }

    public void periodic() {
        controller.setPID(p, i, d);
        int pivotPos = getPosition();


        double pid = controller.calculate(pivotPos, target);
        double ff = calculateFeedForward(pivotPos, target);

        //Total power is the sum of the PID control and the feedforward compensation
        power = pid + ff;

        //Set motor power
        pivotMotor.set(power);
    }

    public double calculateFeedForward(double pos, double setpoint) {
        //convert from encoder ticks to degrees
        double angleInDegrees = pos / ticksPerDegree;
        double angleInRadians = Math.toRadians(angleInDegrees);

        // Effective length of the extension (add this dynamically based on your extension)
        double effectiveExtensionLength = EXTENSION_OFFSET + (Math.toRadians(slides.getPosition() / 537.7) * 20); // calculates ext length

        // Torque from the static (non extending) arm mass (assumed as a uniform rod, the mass is centered at length/3 due to motors being closer to pivot point)
        double armGravityTorque = ARM_MASS * GRAVITY * (ARM_LENGTH / 2.0) * Math.sin(angleInRadians);

        // Torque from the extending arm
        double extensionGravityTorque = EXTENSION_MASS * GRAVITY * (effectiveExtensionLength / 2.0) * Math.sin(angleInRadians);

        // Total gravity compensation torque
        double totalFeedforwardPower = armGravityTorque + extensionGravityTorque;

        return totalFeedforwardPower;
    }

    public int getPosition() {
        // Get the current motor encoder position
        return pivotMotor.getCurrentPosition();
    }

    public double getCurrent() {
        return pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public Pivot.Position getState() {
        return position;
    }

}
