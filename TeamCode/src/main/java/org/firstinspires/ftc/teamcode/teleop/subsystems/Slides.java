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

    public double heightConstant;

    public enum Position {
        // HIGH, MID, LOW, BOTTOM - Position on lines on backboard
        HIGH,
        MID,
        LOW,
        BOTTOM, //storage and outtake round position
    }

    public Position position = Position.BOTTOM;
    public static double p = 0.015, i = 0, d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    private double manualPower = 0;

    public double power;
    public static int high = -2500, mid = -1500, low = -700, bottom = 0;
    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(30000, 20000);

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

    public void runTo(double pos) {
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init_new_profile(motorLeft.getCurrentPosition(), pos);
        profile_init_time = opMode.time;

        goingDown = pos > target;
        target = pos;
    }

    public void runToTop() {
        runTo(high);
        position = Position.HIGH;
    }

    public void runToMiddle() {
        runTo(mid);
        position = Position.MID;
    }

    public void runToLow() {
        runTo(low);
        position = Position.LOW;
    }

    public void runToBottom() {
        runTo(bottom);
        position = Position.BOTTOM;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void resetEncoder() {
        motorLeft.resetEncoder();
        motorRight.resetEncoder();
    }

    public void periodic() {
        motorRight.setInverted(false);
        motorLeft.setInverted(true);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            power = powerUp * controller.calculate(motorLeft.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(motorLeft.getCurrentPosition());
            }
            motorLeft.set(power);
            motorRight.set(power);
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(motorLeft.getCurrentPosition());
                motorLeft.set(manualPower / manualDivide);
                motorRight.set(manualPower / manualDivide);
            } else {
                power = staticF * controller.calculate(motorLeft.getCurrentPosition());
                motorLeft.set(power);
                motorRight.set(power);
            }
        }
    }


    public double getCurrent() {
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }


    public int getPosition() {
        return motorLeft.getCurrentPosition();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(30000, 20000);
    }

    public Position getState() {
        return position;
    }
}
