package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;


@Config
public class Slides {

    public final MotorEx motorLeft;
    public final MotorEx motorRight;
    private final Servo shifter;
    private PIDFController controller;

    public static double p = 0.04, i = 0, d = 0.0012, f = 0, staticFOffset = 0.07, gComp = 0.22;
    public static double staticF = 0;
    public static double ikMMoffset = 200;
    public static double neutral = 0.60, high = 0.79, low = 0.41;

    public boolean climbingPower = false;
    private final double tolerance = 10;
    private final double powerUp = 0.16;
    private double powerDown = 0.05;
    private final double manualDivide = 2;
    private final double powerMin = 0.1;
    public double manualPower = 0;

    public static double coax = 0;

    public double spoolRadiusGear = 9, spoolRadius = 19;
    public static double ticksPerRev = 145.1;

    public double power;
    private final OpMode opMode;
    private double target = 0;
    public boolean goingDown = false;
    private double profile_init_time = 0;

    public static double maxVelo = 40000, maxAccel = 20000;
    private MotionProfiler profiler = new MotionProfiler(maxVelo, maxAccel);
    public boolean profiling;

    public enum SlidesState {
        HIGH,
        HIGH2LOW,
        LOW2HIGH,
        LOW
    }

    public SlidesState state = SlidesState.HIGH;

    public Slides(OpMode opMode) {
        motorLeft = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_1150);
        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_1150);
        shifter =  opMode.hardwareMap.servo.get("shifter");;

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

    private void adjustCoax(double pivotAngleRadians) {
        // Adjust coax based on the pivot angle
        coax = -ticksPerRev * (getGearing()) * ((Math.toDegrees(pivotAngleRadians) - Pivot.startingAngleOffsetDegrees) / 360); //forward of starting position is negative (front intake), back of it is positive (rear)
    }

    public double getCoax() {
        return coax;
    }

    public void runTo(double pos) {
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        if (manualPower == 0) {
            resetProfiler();
            profiler.init_new_profile(getPosition(), pos);
            profile_init_time = opMode.time;
            profiling = true;
        }
        goingDown = pos > target;
        target = pos;
    }

    public void runToIKMM(double posMM) {
        posMM -= ikMMoffset; //length difference from pivot to diffy arm joint
        runToMM(posMM);
    }

    public void runToMM(double posMM) {
        posMM = Math.max(posMM, 0);
        posMM = Math.min(posMM, 720);
        runTo(convert2Ticks(posMM));
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = -manual;
        } else {
            manualPower = 0;
        }
    }

    public void periodic(double pivotAngleRadians) {
        spoolRadiusGear = spoolRadius * (1.0 / getGearing());
        if (state == SlidesState.HIGH || state == SlidesState.LOW) {
            motorRight.setInverted(false);
            motorLeft.setInverted(true);
            controller.setPIDF(p, i, d, f);
            adjustStaticF(pivotAngleRadians);
            adjustCoax(pivotAngleRadians);
            double dt = opMode.time - profile_init_time;
            if (!profiler.isOver()) {
                controller.setSetPoint(profiler.motion_profile_pos(dt) + coax);
                power = powerUp * controller.calculate(getPosition());
                if (goingDown) {
                    powerDown = (powerUp) - (0.01 * Math.sin(pivotAngleRadians));
                    power = powerDown * controller.calculate(getPosition());
                }
            } else {
                if (profiler.isDone()) {
                    resetProfiler();
                    profiling = false;
                }
                if (manualPower != 0) {
                    controller.setSetPoint(getPosition());
                    power = manualPower / manualDivide;
                    target = getPosition() - coax;
                } else {
                    if ((getTargetMM() < 0.5 && getTargetMM() > -0.5 && state != SlidesState.LOW)) {
                        power = 0;
                    } else {
                        controller.setSetPoint(target + coax);
                        power = staticF * controller.calculate(getPosition());
                    }
                }
            }
            setPower(power);
        }
    }

    public void neutral() {
        shifter.setPosition(neutral);
    }

    public void high() {
        shifter.setPosition(high);
    }

    public void low() {
        shifter.setPosition(low);
    }

    public double getCurrent() {
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getPosition() {
        return (motorLeft.getCurrentPosition());
    }

    public double getmmPosition() {
        return Math.toRadians(getPosition() * 360 / -ticksPerRev) * spoolRadiusGear;
    }

    public double getIKmmPosition() {
        return Math.toRadians(getPosition() * 360 / -ticksPerRev) * spoolRadiusGear + ikMMoffset;
    }

    public double convert2MM(double ticks) {
        return Math.toRadians(ticks * 360 / -ticksPerRev) * spoolRadiusGear;
    }

    public double convert2Ticks(double mm) {
        return Math.toDegrees(mm/spoolRadiusGear) * -ticksPerRev / 360;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetMM() {
        return convert2MM(target);
    }

    public double getGearing() {
        if (state == SlidesState.LOW || state == SlidesState.LOW2HIGH)
            return -7.2;
        else {
            return 5.0 / 4.0;
        }
    }

    public void setState(SlidesState state) {
        this.state = state;
        if (state == SlidesState.LOW) {
            p = 0.04;
            d = 0.0012;
            controller.setP(p);
            controller.setD(d);
        } else {
            p = 0.04;
            d = 0.0012;
            controller.setP(p);
            controller.setD(d);
        }
    }

    public double getControllerSetpoint() {
        return controller.getSetPoint();
    }

    public double getError() {
        return controller.getPositionError();
    }

    public void setPower(double power) {
        motorLeft.set(power);
        motorRight.set(power);
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(maxVelo, maxAccel);
    }

    public void resetEncoders() {
        motorLeft.resetEncoder();
        motorRight.resetEncoder();
    }
}
