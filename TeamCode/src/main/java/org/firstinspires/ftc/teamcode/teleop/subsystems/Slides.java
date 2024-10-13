package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

// TODO: Set values for slide positions

@Config
public class Slides {

    public final MotorEx motorLeft;
    public final MotorEx motorRight;
    private PIDFController controller;
    private Pivot pivot;

    public enum Position {
        // HIGH, MID, LOW, BOTTOM - Position on lines on backboard
        HIGH,
        MID,
        LOW,
        BOTTOM, //storage and outtake round position
    }

    public Position position = Position.BOTTOM;
    public static double p = 0.04, i = 0, d = 0.0012, f = 0, staticFOffset = 0.16, gComp = 0.27-staticFOffset;
    public static double staticF = 0;
    private final double tolerance = 15;
    private final double powerUp = 0.1;
    private double powerDown = 0.05;
    private final double manualDivide = 1;
    private final double powerMin = 0.1;
    private double manualPower = 0;

    public double power;
    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profile_init_time = 0;

    public static double maxVelo = 30000, maxAccel = 20000;
    private MotionProfiler profiler = new MotionProfiler(maxVelo, maxAccel);

    public Slides(OpMode opMode) {
        motorLeft = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_312);
        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_312);

        motorRight.setInverted(true);
        motorLeft.setInverted(false);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);

        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.opMode = opMode;
    }

    private void adjustStaticF(double pivotAngleRadians) {
        // Adjust staticF based on the pivot angle
        staticF = gComp * Math.sin(pivotAngleRadians) + staticFOffset;
    }

    public void runTo(double pos) {
        if (pos != target) {
            motorLeft.setRunMode(Motor.RunMode.RawPower);
            motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            motorRight.setRunMode(Motor.RunMode.RawPower);
            motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            controller = new PIDFController(p, i, d, f);
            controller.setTolerance(tolerance);
            if (manualPower == 0) {
                resetProfiler();
                profiler.init_new_profile(motorLeft.getCurrentPosition(), pos);
                profile_init_time = opMode.time;
            }
            goingDown = pos > target;
            target = pos;
        }
    }

    public void runRelativeMM(double mm) {
        runToMM(getmmPosition() + mm);
    }

    public void runToMM(double posMM) {
        //posMM -= 280; //length difference from pivot to diffy arm joint TODO: uncomment for IK
        posMM = Math.max(posMM, 0);
        posMM = Math.min(posMM, 720);
        runTo(convert2Ticks(posMM));
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = manual;
            runRelativeMM(manual * 5);
        } else {
            manualPower = 0;
        }
    }

    public void resetEncoder() {
        motorLeft.resetEncoder();
        motorRight.resetEncoder();
    }

    public void periodic(double pivotAngleRadians) {
        motorRight.setInverted(false);
        motorLeft.setInverted(true);
        controller.setPIDF(p, i, d, f);
        adjustStaticF(pivotAngleRadians);

        if (manualPower == 0) {
            double dt = opMode.time - profile_init_time;
            if (!profiler.isOver()) {
                controller.setSetPoint(profiler.motion_profile_pos(dt));
                power = powerUp * controller.calculate(motorLeft.getCurrentPosition());
                if (goingDown) {
                    powerDown = powerUp - (0.05 * Math.sin(pivotAngleRadians));
                    power = powerDown * controller.calculate(motorLeft.getCurrentPosition());
                }
            } else {
                if (profiler.isDone()) {
                    resetProfiler();
                }
                power = staticF * controller.calculate(motorLeft.getCurrentPosition());
            }
        } else {
            power = controller.calculate(motorLeft.getCurrentPosition(), target);
        }
        motorLeft.set(power);
        motorRight.set(power);
    }

    public double getCurrent() {
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }


    public double getPosition() {
        return motorLeft.getCurrentPosition();
    }

    public double getmmPosition() {
        return Math.toRadians(getPosition() * 360 / -537.7) * 20;
    }

    public double convert2MM(double ticks) {
        return Math.toRadians(ticks * 360 / -537.7) * 20;
    }

    public double convert2Ticks(double mm) {
        return Math.toDegrees(mm/20) * -537.7 / 360;
    }

    public double getTarget() {
        return target;
    }

    public double getProfilerTarget() {
        return controller.getSetPoint();
    }

    public double getError() {
        return controller.getPositionError();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(maxVelo, maxAccel);
    }

    public Position getState() {
        return position;
    }
}
