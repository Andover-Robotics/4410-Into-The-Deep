package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        WALL_INTAKE, //specimen intake from wall
        STORAGE, //starting config (18x18x18)
        CLIMBING
    }

    public boolean first, second, third, fourth, fifth;

    public static Bot instance;
    public BotState state = BotState.STORAGE; // Default bot state
    private final MotorEx fl, fr, bl, br;
    public OpMode opMode;
    public double heading = 0.0;
    private MecanumDrive drive;

    // Define subsystem objects
    public Gripper gripper;
    public Pivot pivot;

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
                gripper.close();
                if (state == BotState.FRONT_INTAKE) {
                    pivot.storage(true, true);
                } else if (state == BotState.REAR_INTAKE) {
                    pivot.arm.rearPickupToStorage();
                    pivot.storage(true, true);
                    Thread.sleep(300);
                    pivot.arm.storage();
                } else if (state == BotState.WALL_INTAKE) {
                    pivot.storage(true, true);
                    Thread.sleep(250);
                    pivot.arm.storage();
                } else if (state == BotState.HIGH_BUCKET) {
                    pivot.storage(false, true);
                    Thread.sleep(900);
                    pivot.storage(true, true);
                    pivot.arm.storage();
                } else if (state == BotState.HIGH_CHAMBER){
                    pivot.storage(false, true);
                    Thread.sleep(500);
                    pivot.storage(true, true);
                    pivot.arm.storage();
                } else {
                    pivot.storage(false, true);
                    Thread.sleep(400);
                    pivot.storage(true, false);
                    pivot.arm.storage();
                }
                state = BotState.STORAGE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void lowBucket() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.HIGH_BUCKET || state == BotState.HIGH_CHAMBER) { //TODO: Test if hits bucket
                    pivot.arm.vertical();
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    Thread.sleep(250);
                }
                pivot.lowBucket(true, false);
                Thread.sleep(300);
                pivot.lowBucket(false, true);
                Thread.sleep(200);
                pivot.arm.bucket();
                state = BotState.LOW_BUCKET;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void highBucket() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.LOW_BUCKET) { //TODO: Test if hits bucket
                    pivot.arm.vertical();
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    pivot.highBucket(true, false);
                    Thread.sleep(400);
                } else {
                    pivot.highBucket(true, false);
                    Thread.sleep(300);
                }
                pivot.highBucket(false, true);
                Thread.sleep(900);
                pivot.arm.bucket();
                state = BotState.HIGH_BUCKET;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionHighBucket() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highBucket(true, false)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.highBucket(false, true)),
                new SleepAction(0.9),
                new InstantAction(() -> pivot.arm.bucket()),
                new InstantAction(() -> state = BotState.HIGH_BUCKET)
        );
    }

    public void lowChamber() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.HIGH_CHAMBER) { //TODO: Test if hits chamber rod
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    Thread.sleep(400);
                }
                pivot.lowChamber(true, false);
                Thread.sleep(200);
                pivot.arm.outtakeUp();
                pivot.lowChamber(false, true);
                state = BotState.LOW_CHAMBER;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void prel2Climb() {
        Thread thread = new Thread(() -> {
            try {
                if (state != BotState.STORAGE) { //TODO: Test if hits chamber rod
                    storage();
                    Thread.sleep(500);
                }
                pivot.arm.outtakeUp();
                pivot.prel2Climb(true, false);
                Thread.sleep(150);
                pivot.prel2Climb(false, true);
                state = BotState.CLIMBING;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void l2Climb() {
        Thread thread = new Thread(() -> {
            try {
                pivot.midl2Climb(false, true);
                Thread.sleep(150);
                pivot.postl2Climb(true, false);
                Thread.sleep(175);
                pivot.postl2Climb(false, true);
                Thread.sleep(1000);
                pivot.climbTransfer(true, false);
                Thread.sleep(1000);
                pivot.climbTransfer(false, true);
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void l3Climb() {
        Thread thread = new Thread(() -> {
            try {
                pivot.climbTransfer(true, false);
                Thread.sleep(1000);
                pivot.climbTransfer(false, true);
                Thread.sleep(1000);
                pivot.prel3Climb(true, false);
                Thread.sleep(1000);
                pivot.prel3Climb(false, true);
                Thread.sleep(1000);
                pivot.midl3Climb(true, false);
                Thread.sleep(1000);
                pivot.midl3Climb(false, true);
                Thread.sleep(1000);
                pivot.tiltedl3Climb(true, false);
                Thread.sleep(1000);
                pivot.tiltedl3Climb(false, true);
                Thread.sleep(1000);
                pivot.backTiltedl3Climb(true, false);
                Thread.sleep(1000);
                pivot.backTiltedl3Climb(false, true);
                Thread.sleep(1000);
                pivot.postl3Climb(false, true);
                Thread.sleep(1000);
                pivot.postl3Climb(true, false);
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void highChamber() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.LOW_CHAMBER) { //TODO: Test if hits chamber rod
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    Thread.sleep(400);
                }
                pivot.highChamber(true, false);
                Thread.sleep(300);
                pivot.highChamber(false, true);
                Thread.sleep(80);
                pivot.arm.outtakeUp();
                state = BotState.HIGH_CHAMBER;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionHighChamber() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highChamber(true, false)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.highChamber(false, true)),
                new SleepAction(0.05),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> state = BotState.HIGH_CHAMBER)
        );
    }

    public void clipDown() {
        pivot.arm.outtakeDown();
    }

    public SequentialAction actionClipDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeDown())
        );
    }

    public void clipCancel() {
        pivot.arm.outtakeUp();
    }

    public void clipStorage() {
        Thread thread = new Thread(() -> {
            try {
                gripper.open();
                Thread.sleep(350);
                pivot.arm.outtakeUp();
                storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionClipStorage() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.35),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.5),
                new InstantAction(() -> pivot.storage(true, true)),
                new InstantAction(() -> pivot.arm.storage()),
                new InstantAction(() -> state = BotState.STORAGE)
        );
    }

    public void bucketDrop() {
        Thread thread = new Thread(() -> {
            try {
                gripper.open();
                Thread.sleep(400);
                pivot.arm.outtakeUp();
                storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionBucketDrop() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.9),
                new InstantAction(() -> pivot.storage(true, true)),
                new InstantAction(() -> pivot.arm.storage()),
                new InstantAction(() -> state = BotState.STORAGE)
        );
    }

    public void frontIntakeToStorage() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(2);
                Thread.sleep(200);
                pivot.storage(false, true);
                pivot.arm.frontPickupToStorage();
                Thread.sleep(400);
                pivot.storage(true, false);
                Thread.sleep(300);
                pivot.arm.storage();
                storage();

            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionFrontIntakeToStorage() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(2)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.storage(false, true)),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.storage(true, false)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.arm.storage()),
                new InstantAction(() -> state = BotState.STORAGE)
        );
    }

    public void frontIntake() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.LOW_CHAMBER | state == BotState.HIGH_CHAMBER) {
                    storage();
                    Thread.sleep(225);
                }
                gripper.open();

                pivot.arm.frontPickupToStorage();
                Thread.sleep(100);
                pivot.frontIntakeStorage(true, false);
                Thread.sleep(100);
                pivot.frontIntake(true, true);
                Thread.sleep(300);
                pivot.arm.frontPickup();
                state = BotState.FRONT_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionFrontIntake() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.frontIntakeStorage(true, true)),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.frontAutoIntake(true, true)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public void pickDown() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(-3.1);
                Thread.sleep(300);
                gripper.close();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionPickDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(-3.3)),
                new SleepAction(0.4),
                new InstantAction(() -> gripper.close())
        );
    }

    public void pickUp() {
        pivot.changeZ(+3.1);
    }

    public SequentialAction actionPickUp() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(3.3))
        );
    }

    public void wallIntakeOpen() {
        Thread thread = new Thread(() -> {
            try {
                gripper.open();
                pivot.wallIntake(true, true);
                Thread.sleep(200);
                pivot.arm.wallPickup();
                state = BotState.WALL_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void wallIntakeClosed() {
        Thread thread = new Thread(() -> {
            try {
                gripper.close();
                pivot.wallIntake(true, true);
                Thread.sleep(200);
                pivot.arm.wallPickup();
                state = BotState.WALL_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionWallIntakeOpen() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.wallIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.arm.wallPickup()),
                new InstantAction(() -> state = BotState.WALL_INTAKE)
        );
    }

    public SequentialAction actionWallIntakeClosed() {
        return new SequentialAction(
                new InstantAction(() -> gripper.close()),
                new InstantAction(() -> pivot.wallIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.arm.wallPickup()),
                new InstantAction(() -> state = BotState.WALL_INTAKE)
        );
    }

    public SequentialAction actionWallIntakeToStorage() {
        return new SequentialAction(
                new InstantAction(() -> pivot.storage(true, true)),
                new SleepAction(0.25),
                new InstantAction(() -> pivot.arm.storage())
        );
    }

    public SequentialAction actionCloseGripper() {
        return new SequentialAction(
                new InstantAction(() -> gripper.close())
        );
    }

    public SequentialAction actionOpenGripper() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open())
        );
    }

    public SequentialAction actionRotateClaw() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.rollLeft()),
                new InstantAction(() -> pivot.arm.rollLeft())
        );
    }

    public class periodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.periodic();
            return true;
        }
    }
    public Action periodic() {
        return new periodic();
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