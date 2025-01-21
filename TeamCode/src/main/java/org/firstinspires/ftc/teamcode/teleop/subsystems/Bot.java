package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

import org.firstinspires.ftc.teamcode.util.SampleDetectionPipeline;


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

    public static int detectionCounter, searchCounter;

    public SampleDetectionPipeline pipeline;

    double pickDownUpValue = 3.75; //TELEOP ONLY (not auton)

    public static double sampleYPos = 0;

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
        sampleYPos = 0;
    }

    public void openPipeline(boolean red, boolean blue, boolean yellow) {
        pipeline = new SampleDetectionPipeline(red, blue, yellow, opMode.hardwareMap);
        detectionCounter = 0;
    }

    public void alignClaw() {
        pivot.arm.setRollPitch(pipeline.getAngle(), pivot.arm.pitchGroundPickup);
    }

    public void scan() {
        pipeline.detect();
    }

    public void resetSampleDrive() {
        sampleYPos = 0; //camera is sideways so X is Y
    }

    public void updateSampleDrive() {
        sampleYPos = pipeline.getX(); //camera is sideways so X is Y
    }

    public void updateSampleSlides() {
        pivot.changeX(pipeline.getY()); //camera is sideways so Y is X
    }
    public class actionDetectWait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if ((sampleYPos != 0 && pipeline.getAngle() != -1 && detectionCounter > 4) || searchCounter > 9) {
                searchCounter = 0;
                return false;
            } else if (sampleYPos != 0 && pipeline.getAngle() != -1) {
                scan();
                updateSampleDrive();
                detectionCounter++;
                searchCounter++;
                return true;
            } else {
                scan();
                updateSampleDrive();
                searchCounter++;
                return true;
            }
        }
    }

    public Action actionDetectWait() {
        return new actionDetectWait();
    }

    public void resetPipeline() {
        resetSampleDrive();
        detectionCounter = 0;
    }

    public Action actionResetPipeline() {
        return new InstantAction(this::resetPipeline);
    }

    public SequentialAction actionDetect() {
        return new SequentialAction(
                new InstantAction(this::updateSampleDrive),
                //new InstantAction(this::updateSampleSlides),
                new InstantAction(this::alignClaw)
        );
    }

    public void subAutoIntake() {
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
                pivot.subAutoIntake(true, true);
                Thread.sleep(300);
                pivot.arm.frontPickup();
                state = BotState.FRONT_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void initialize() {
        storage();
        pivot.slides.high();
    }

    public void storage() {
        Thread thread = new Thread(() -> {
            try {
                gripper.close();
                if (state == BotState.FRONT_INTAKE) {
                    pivot.storage(true, true);
                } else if (state == BotState.WALL_INTAKE) {
                    pivot.storage(true, true);
                    Thread.sleep(250);
                    pivot.arm.storage();
                } else if (state == BotState.HIGH_BUCKET) {
                    pivot.arm.outtakeUp();
                    pivot.storage(false, true);
                    Thread.sleep(450);
                    pivot.storage(true, true);
                    pivot.arm.storage();
                } else if (state == BotState.HIGH_CHAMBER){
                    pivot.highChamberTransfer(false, true);
                    Thread.sleep(50);
                    pivot.arm.outtakeHoriz();
                    pivot.highChamberTransfer(true, false);
                    Thread.sleep(400);
                    pivot.arm.outtakeUp();
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

    public void storageOpenGripper() {
        Thread thread = new Thread(() -> {
            try {
                gripper.open();
                if (state == BotState.FRONT_INTAKE) {
                    pivot.storage(true, true);
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
                    pivot.highChamberTransfer(false, true);
                    Thread.sleep(50);
                    pivot.arm.outtakeHoriz();
                    pivot.highChamberTransfer(true, false);
                    Thread.sleep(400);
                    pivot.arm.outtakeUp();
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

    public void shiftDown() {
        Thread thread = new Thread(() -> {
            try {
                pivot.slides.state = Slides.SlidesState.HIGH2LOW;
                pivot.goToResetPosition();
                Thread.sleep(250);
                pivot.slides.setPower(0);
                pivot.slides.neutral();
                Thread.sleep(300);
                pivot.slides.setPower(-0.1); //Upwards (for high) to help it disengage
                Thread.sleep(75);
                pivot.slides.setPower(0);
                pivot.slides.low(); //Engage high
                while (pivot.slides.getCurrent() < 1000) {
                    pivot.slides.setPower(-0.5); //Downwards (for low) to help it engage
                }
                Thread.sleep(200);
                pivot.slides.setPower(0.3);
                Thread.sleep(100);
                pivot.slides.setPower(-1); //run it into the ground
                Thread.sleep(200);
                pivot.slides.setPower(0);
                pivot.slides.resetEncoders();
                pivot.slides.state = Slides.SlidesState.LOW;
            } catch (InterruptedException ignored) {}
        });
        if (pivot.slides.state == Slides.SlidesState.HIGH)
            thread.start();
    }

    public void shiftUp() {
        Thread thread = new Thread(() -> {
            try {
                pivot.slides.state = Slides.SlidesState.LOW2HIGH;
                pivot.goToResetPosition();
                Thread.sleep(250);
                pivot.slides.setPower(0);
                pivot.slides.neutral();
                Thread.sleep(300);
                pivot.slides.setPower(0.1); //Upwards (for high) to help it disengage
                Thread.sleep(75);
                pivot.slides.setPower(0);
                pivot.slides.high(); //Engage high
                while (pivot.slides.getCurrent() < 1000) {
                    pivot.slides.setPower(0.1); //Downwards (for low) to help it engage
                }
                Thread.sleep(200);
                pivot.slides.setPower(-0.3);
                Thread.sleep(100);
                pivot.slides.setPower(1); //run it into the ground
                Thread.sleep(100);
                pivot.slides.setPower(0);
                pivot.slides.resetEncoders();
                pivot.slides.state = Slides.SlidesState.HIGH;
            } catch (InterruptedException ignored) {}
        });
        if (pivot.slides.state == Slides.SlidesState.LOW)
            thread.start();
    }

    public void lowBucket() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.HIGH_BUCKET || state == BotState.HIGH_CHAMBER) {
                    pivot.arm.vertical();
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    Thread.sleep(250);
                }
                pivot.lowBucket(true, false);
                Thread.sleep(300);
                pivot.lowBucket(false, true);
                Thread.sleep(125);
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
                    Thread.sleep(250);
                } else {
                    pivot.highBucket(true, false);
                    Thread.sleep(300);
                }
                pivot.highBucket(false, true);
                Thread.sleep(650);
                pivot.arm.bucket();
                state = BotState.HIGH_BUCKET;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void lowChamber() {
        Thread thread = new Thread(() -> {
            try {
                if (state == BotState.HIGH_CHAMBER) { //TODO: Test if hits chamber rod
                    pivot.storage(false, true); //pull slides in so that it doesn't hit
                    Thread.sleep(400);
                } else if (state == BotState.HIGH_BUCKET) {
                    storage();
                    Thread.sleep(600);
                }
                pivot.lowChamber(true, false);
                Thread.sleep(250);
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
                pivot.arm.bucket();
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
                Thread.sleep(1650);
                pivot.climbTransfer(true, false);
                Thread.sleep(750);
                pivot.climbTransfer(false, true);
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void l3Climb() {
        Thread thread = new Thread(() -> {
            try {
                pivot.prel3Climb(true, false);
                Thread.sleep(600);
                pivot.prel3Climb(false, true);
                Thread.sleep(1700);
                pivot.midl3Climb(true, false);
                Thread.sleep(400);
                pivot.arm.outtakeHoriz();
                Thread.sleep(150);
                pivot.midl3Climb(false, true);
                Thread.sleep(1000);
                pivot.tiltedl3Climb(true, false);
                Thread.sleep(800);
                pivot.tiltedl3Climb(false, true);
                Thread.sleep(1700);
                pivot.backTiltedl3Climb(true, false);
                Thread.sleep(1300);
                pivot.postl3Climb(false, true);
                Thread.sleep(1000);
                pivot.postl3Climb(true, false);
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
                } else if (state == BotState.HIGH_BUCKET) {
                    storage();
                    Thread.sleep(600);
                }
                pivot.arm.outtakeUp();
                pivot.highChamber(true, false);
                Thread.sleep(200);
                pivot.highChamber(false, true);
                state = BotState.HIGH_CHAMBER;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }


    public void clipCancel() {
        pivot.arm.outtakeUp();
    }

    public void clipDown() {
        pivot.arm.outtakeDown();
    }

    public void clipStorage() {
        Thread thread = new Thread(() -> {
            try {
                Thread.sleep(200);
                gripper.open();
                Thread.sleep(350);
                pivot.arm.outtakeUp();
                storageOpenGripper();
                Thread.sleep(300);
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void bucketDrop() {
        Thread thread = new Thread(() -> {
            try {
                gripper.open();
                Thread.sleep(400);
                storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void frontIntakeToStorage() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(2);
                Thread.sleep(200);
                pivot.storage(false, true);
                pivot.arm.frontPickupToStorage();
                Thread.sleep(300);
                pivot.storage(true, false);
                Thread.sleep(300);
                pivot.arm.storage();
                storage();

            } catch (InterruptedException ignored) {}
        });
        thread.start();
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

    public void pickDown() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(-pickDownUpValue);
                Thread.sleep(200);
                gripper.close();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void subAutoPickDown() {
        Thread thread = new Thread(() -> {
            try {
                pivot.changeZ(-5.5);
                Thread.sleep(500);
                pivot.changeZ(-4.5);
                Thread.sleep(400);
                gripper.close();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionSubAutoPickDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeXZ(pipeline.getY(), -6.1)),//7.1
                new SleepAction(0.3),
                new InstantAction(() -> pivot.changeZ(-4.5)),//3.5
                new SleepAction(0.3),
                new InstantAction(() -> gripper.close()),
                new SleepAction(0.1)
        );
    }

    public void subAutoPickUp() {
        pivot.changeZ(4);
    }


    public SequentialAction actionSubAutoPickUp() {
        return new SequentialAction(
                new InstantAction(this::subAutoPickUp)
        );
    }

    public SequentialAction actionSubAutoIntake() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new InstantAction(() -> pivot.frontIntakeStorage(true, false)),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.subAutoIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.arm.cv())
        );
    }

    public void pickUp() {
        pivot.changeZ(pickDownUpValue);
    }

    public void wallIntakeOpen() {
        Thread thread = new Thread(() -> {
            try {
                pivot.teleopWallIntake(true, true);
                Thread.sleep(200);
                pivot.arm.wallPickup();
                gripper.open();
                state = BotState.WALL_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void wallIntakeClosed() {
        Thread thread = new Thread(() -> {
            try {
                gripper.close();
                pivot.teleopWallIntake(true, true);
                Thread.sleep(200);
                pivot.arm.wallPickup();
                state = BotState.WALL_INTAKE;
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public SequentialAction actionSubPark() {
        return new SequentialAction(
                new InstantAction(() -> pivot.subPark(true, true)),
                new InstantAction(() -> pivot.arm.vertical())
        );
    }

    public SequentialAction actionHighBucket() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highBucket(true, false)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.highBucket(false, true)),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new SleepAction(0.6),
                new InstantAction(() -> pivot.arm.bucket()),
                new InstantAction(() -> state = BotState.HIGH_BUCKET)
        );
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

    public SequentialAction actionWallToHighChamber() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highChamber(true, false)),
                new SleepAction(0.6),
                new InstantAction(() -> pivot.highChamber(false, true)),
                new SleepAction(0.05),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> state = BotState.HIGH_CHAMBER)
        );
    }

    public SequentialAction actionPickUp() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(5))
        );
    }

    public SequentialAction actionPickDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(-4)),
                new SleepAction(0.25),
                new InstantAction(() -> gripper.close()),
                new SleepAction(0.075)
        );
    }

    public SequentialAction actionClipStorage() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.arm.outtakeHoriz()),
                new InstantAction(() -> pivot.highChamberTransfer(false, true)),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.highChamberTransfer(true, false)),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.5),
                new InstantAction(() -> pivot.storage(true, true)),
                new InstantAction(() -> pivot.arm.storage())
        );
    }

    public SequentialAction actionFirstClipStorage() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.arm.horizLower()),
                new InstantAction(() -> pivot.highChamberTransfer(false, true)),
                new InstantAction(() -> pivot.highChamberTransfer(true, false))

        );
    }

    public SequentialAction actionSecondClipStorage() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.horizLower()),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.storage(true, true)),
                new InstantAction(() -> pivot.arm.storage())
                );
    }

    public SequentialAction actionClipWall() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.horizLower()),
                new InstantAction(() -> pivot.highChamberTransfer(false, true)),
                new InstantAction(() -> pivot.highChamberTransfer(true, false)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.autoWallIntake(false, true)),
                new InstantAction(() -> pivot.arm.wallPickup()),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.autoWallIntake(true, false)),
                new InstantAction(() -> state = BotState.WALL_INTAKE)
        );
    }

    public SequentialAction actionBucketDrop() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.20),
                new InstantAction(() -> pivot.arm.outtakeUp())
        );
    }

    public SequentialAction actionBucketToStorage() {
        return new SequentialAction(
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.storage(true, true)),
                new InstantAction(() -> pivot.arm.storage()),
                new InstantAction(() -> state = BotState.STORAGE)
        );
    }

    public SequentialAction actionFrontIntakeToStorage() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(3)),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.storage(true, false)),
                new SleepAction(0.5),
                new InstantAction(() -> pivot.arm.storage()),
                new InstantAction(() -> state = BotState.STORAGE)
        );
    }

    public SequentialAction actionIntakeToHighBucket() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highBucket(true, false)),
                new SleepAction(0.35),
                new InstantAction(() -> pivot.highBucket(false, true)),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new SleepAction(0.55),
                new InstantAction(() -> pivot.arm.bucket()),
                new InstantAction(() -> state = BotState.HIGH_BUCKET)
        );
    }

    public SequentialAction actionBucketToFrontIntake() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.frontAutoIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionPushIntake() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeHoriz()),
                new InstantAction(() -> pivot.pushIntake(true, false)),
                new InstantAction(() -> pivot.arm.outtakeHoriz()),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.pushIntake(false, true)),
                new SleepAction(0.4),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionFrontIntake() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.frontAutoIntake(true, true)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionSecondClipToIntake() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new InstantAction(() -> pivot.frontAutoIntake(true, false)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionDiagFrontIntake() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.frontAutoIntake(true, false)),
                new SleepAction(0.25),
                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
                new SleepAction(0.3),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionWallIntakeOpen() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.autoWallIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.arm.wallPickup()),
                new InstantAction(() -> state = BotState.WALL_INTAKE)
        );
    }

    public SequentialAction actionWallIntakeClosed() {
        return new SequentialAction(
                new InstantAction(() -> gripper.close()),
                new InstantAction(() -> pivot.autoWallIntake(true, true)),
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
                new InstantAction(() -> pivot.arm.rollLeft()),
                new InstantAction(() -> pivot.arm.rollLeft())
        );
    }

    public SequentialAction actionHPRotateClaw() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.rollLeft())
        );
    }

    public class actionPeriodic implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.periodic();
            return true;
        }
    }

    public Action actionPeriodic() {
        return new actionPeriodic();
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
        double frontWheelModifier = (state == BotState.FRONT_INTAKE)? 1.15 : 1.03;
        if (state == BotState.HIGH_CHAMBER) frontWheelModifier = 1.2;
        double rearWheelModifier = (state == BotState.WALL_INTAKE || state == BotState.HIGH_BUCKET || state == BotState.LOW_BUCKET)? 1.15 : 1;
        double[] speeds = {
                (forwardBackSpeed - strafeSpeed - turnSpeed) * frontWheelModifier,
                (forwardBackSpeed + strafeSpeed + turnSpeed) * frontWheelModifier,
                (forwardBackSpeed + strafeSpeed - turnSpeed) * rearWheelModifier,
                (forwardBackSpeed - strafeSpeed + turnSpeed) * rearWheelModifier
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