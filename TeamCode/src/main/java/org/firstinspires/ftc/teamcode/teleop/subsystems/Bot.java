package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Bot {
    public enum BotState {
        HIGH_BUCKET, //high bucket
        LOW_BUCKET, //low bucket
        HIGH_CHAMBER, //high chamber
        LOW_CHAMBER, //low chamber
        REAR_INTAKE, //rear intake
        FRONT_INTAKE, //front intake
        WALL_INTAKE,
        STORAGE, //starting config (18x18x18)
    }

    public static Bot instance;
    public BotState state = BotState.STORAGE; // Default bot state
    private final MotorEx fl, fr, bl, br;
    public OpMode opMode;
    public double heading = 0.0;
    private MecanumDrive drive;

    // Define subsystem objects
    public Gripper gripper;
    public Pivot pivot;

    public double wristUpPos = 0.0;


    // get bot instance
    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized!");
        }
        return instance;
    }
    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode) {
        this.opMode = opMode;

        fl = new MotorEx(opMode.hardwareMap, "motorFL", Motor.GoBILDA.RPM_435);
        fr = new MotorEx(opMode.hardwareMap, "motorFR", Motor.GoBILDA.RPM_435);
        bl = new MotorEx(opMode.hardwareMap, "motorBL", Motor.GoBILDA.RPM_435);
        br = new MotorEx(opMode.hardwareMap, "motorBR", Motor.GoBILDA.RPM_435);

        drive = new MecanumDrive(fl, fr, bl, br);

        gripper = new Gripper(opMode);
        pivot = new Pivot(opMode);
    }

    public void storage() {
        Thread thread = new Thread(() -> {
            try {
                pivot.storage();
                Thread.sleep(25);
                pivot.arm.storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void lowBucket() {
        Thread thread = new Thread(() -> {
            try {
                pivot.lowBucket();
                Thread.sleep(25);
                pivot.arm.bucket();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void lowChamber() {
        Thread thread = new Thread(() -> {
            try {
                pivot.lowChamber();
                Thread.sleep(25);
                pivot.arm.outtakeUp();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void highChamber() {
        Thread thread = new Thread(() -> {
            try {
                pivot.highChamber();
                Thread.sleep(25);
                pivot.arm.outtakeUp();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void clip() {
        Thread thread = new Thread(() -> {
            try {
                pivot.arm.outtakeDown();
                Thread.sleep(100);
                gripper.outtake();
                Thread.sleep(100);
                storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void frontIntakeToStorage() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(4);
                Thread.sleep(200);
                pivot.frontIntakeStorage();
                Thread.sleep(400);
                pivot.storage();
                Thread.sleep(150);
                pivot.arm.storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void toFrontIntake() {
        Thread thread = new Thread(() -> {
            try {
                pivot.arm.horizontal();
                Thread.sleep(100);
                pivot.frontIntakeStorage();
                Thread.sleep(200);
                pivot.frontIntake();
                Thread.sleep(200);
                pivot.arm.pickup();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void pickDown() {
        pivot.changeZ(-1.5);
    }

    public void toRearIntake() {
        Thread thread = new Thread(() -> {
            try {
                pivot.rearIntake();
                Thread.sleep(200);
                pivot.arm.pickup();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void wallIntake() {
        Thread thread = new Thread(() -> {
            try {
                pivot.wallIntake();
                Thread.sleep(200);
                pivot.arm.horizontalRear();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    // MOTORS
    public void fixMotors() {
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fl.setInverted(false);
        fr.setInverted(true);
        bl.setInverted(false);
        br.setInverted(true);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
    }
    public void stopMotors() {
        fl.set(0.0);
        fr.set(0.0);
        bl.set(0.0);
        br.set(0.0);
    }
    public double getMotorCurrent() {
        return fl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + fr.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + bl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + br.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // DRIVE METHODS
    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }
    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double magnitude = Math.sqrt(strafeSpeed * strafeSpeed + forwardBackSpeed * forwardBackSpeed);
        double theta = (Math.atan2(forwardBackSpeed, strafeSpeed) - heading) % (2 * Math.PI);
        double[] speeds = {
                magnitude * Math.sin(theta + Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) - turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta + Math.PI / 4) - turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }

    public void setHeading (double heading) {
        this.heading = heading;
    }
}