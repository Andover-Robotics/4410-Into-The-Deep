package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

@Config
public class Pivot {

    private PIDFController controller;
    public final MotorEx pivotMotor;

    public double heightConstant;

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

    public Pivot(OpMode opMode) {
        pivotMotor = new MotorEx(opMode.hardwareMap, "pivot", Motor.GoBILDA.RPM_30);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);

        this.opMode = opMode;
    }

    public void runTo(double pos) {
        pivotMotor.setRunMode(Motor.RunMode.RawPower);
        pivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init_new_profile(pivotMotor.getCurrentPosition(), pos);
        profile_init_time = opMode.time;

        goingDown = pos > target;
        target = pos;
    }

    public void runToIn() {
        runTo(bottom);
        position = Pivot.Position.IN;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    public void resetEncoder() {
        pivotMotor.resetEncoder();
    }

    public void periodic() {
        pivotMotor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            power = powerUp * controller.calculate(pivotMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(pivotMotor.getCurrentPosition());
            }
            pivotMotor.set(power);
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(pivotMotor.getCurrentPosition());
                pivotMotor.set(manualPower / manualDivide);
            } else {
                power = staticF * controller.calculate(pivotMotor.getCurrentPosition());
                pivotMotor.set(power);
            }
        }
    }


    public double getCurrent() {
        return pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }


    public int getPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(30000, 20000);
    }

    public Pivot.Position getState() {
        return position;
    }

}
