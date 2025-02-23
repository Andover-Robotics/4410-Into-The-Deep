package org.firstinspires.ftc.teamcode.teleop.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

import org.firstinspires.ftc.teamcode.util.SampleDetectionPipeline;

import java.util.ArrayList;
import java.util.List;


public class Bot {
    public enum BotState {
        HIGH_BUCKET, //high bucket
        LOW_BUCKET, //low bucket
        HIGH_CHAMBER, //high chamber
        LOW_CHAMBER, //low chamber
        FRONT_INTAKE, //front intake
        WALL_INTAKE, //specimen intake from wall
        STORAGE, //starting config (18x18x18)
        CLIMBING,
        RESETTING
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
    public static int angleOffset = 0;
    public static double ySign = -1;
    public static double xSign = 1;
    public double refAngle = 0, x = 0, y = 0;
    private boolean breakBeamWorking = true;
    public boolean climbing = false;

    // Define subsystem objects
    public Gripper gripper;
    public Pivot pivot;

    DigitalChannel breakBeam;
    public boolean holding;

    public static Pose2d storedPosition = null;
    public static Vector2d targetPosition = new Vector2d(0, 5);

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
        breakBeam = opMode.hardwareMap.get(DigitalChannel.class, "BreakBeam");
    }

    public boolean isHolding() {
        return !breakBeam.getState(); //TODO uncomment when bb is fixed
//        return true; //hardcoded holding
    }

    public boolean isEmpty() {
        if (breakBeamWorking) {
            return breakBeam.getState(); //TODO uncomment when bb is fixed
        } else {
            return false;
        }

//        return false; //hardcoded holding
    }

    public void checkBreakBeam(boolean correct) {
        if (breakBeam.getState() != correct) {
            breakBeamWorking = false;
        }
    }

    public void fixBreakBeam() {
        breakBeamWorking = true;
    }

    public void turnOffBreakBeam() {
        breakBeamWorking = false;
    }

    public void saveBreakBeam() {
        holding = isHolding();
    }

    //SAMPLE DETECTION

    public void openPipeline(boolean red, boolean blue, boolean yellow) {
        pipeline = new SampleDetectionPipeline(red, blue, yellow, opMode.hardwareMap);
        detectionCounter = 0;
    }

    public void alignClaw() {
        pivot.arm.setRollPitch(((pipeline.getAngle() - 180) * 1.13) + 180, pivot.arm.pitchGroundPickup);
    }

    public void scan() {
        pipeline.detect();
    }

    public void resetSampleDrive() {
        sampleYPos = 0; //camera is sideways so X is Y
        angleOffset = 0;
    }

    public void updateSampleDrive() {
        sampleYPos = pipeline.getDriveX(); //camera is sideways so X is Y
        angleOffset = (int) 0;//Math.round(-sampleYPos / 2.5);
    }

    public void updateSampleSlides() {
        pivot.changeX(pipeline.getSlidesY()); //camera is sideways so Y is X
    }

    public void savePosition(Pose2d current) {
        storedPosition = current;
        refAngle = Math.toDegrees(current.component2().toDouble()) % 360;
        x = Math.sin(Math.toRadians(refAngle)) * sampleYPos * xSign;
        y = Math.cos(Math.toRadians(refAngle)) * sampleYPos * ySign;
        targetPosition = new Vector2d(storedPosition.component1().x + x, storedPosition.component1().y + y);
    }

    public static Pose2d getCurrentPosition() {
        return storedPosition;
    }

    public static Vector2d getTargetPosition() {
        return targetPosition;
    }

    public class actionDetectWait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if ((sampleYPos != 0 && pipeline.getAngle() != -1 && detectionCounter > 5) || searchCounter > 14) {
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
        searchCounter = 0;
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

    //TELEOP ACTIONS

    public void teleopGoToReset() {
        pivot.goToResetPosition();
        state = BotState.RESETTING;
    }

    public void teleopResetSlides() {
        pivot.slides.resetEncoders();
    }

    public void teleopInitialize() {
        teleopStorage();
        pivot.slides.high();
    }

    public void teleopPivotStorage() {
        pivot.storage(true, true);
        state = BotState.STORAGE;
    }

    public SequentialAction teleopStorage() {
        List<Action> actions = new ArrayList<>();

        actions.add(new InstantAction(() -> gripper.close()));

        if (state == BotState.FRONT_INTAKE) {
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
        } else if (state == BotState.WALL_INTAKE) {
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.HIGH_BUCKET) {
            actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.45));
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.LOW_BUCKET) {
            actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.HIGH_CHAMBER) {
//            actions.add(new InstantAction(() -> pivot.highChamberTransfer(false, true)));
//            actions.add(new SleepAction(0.05));
//            actions.add(new InstantAction(() -> pivot.arm.outtakeHoriz()));
//            actions.add(new InstantAction(() -> pivot.highChamberTransfer(true, false)));
//            actions.add(new SleepAction(0.4));
//            actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
//            actions.add(new InstantAction(() -> pivot.storage(false, true)));
//            actions.add(new SleepAction(0.5));
//            actions.add(new InstantAction(() -> pivot.storage(true, true)));
//            actions.add(new InstantAction(() -> pivot.arm.storage()));

            //for SLIDES CLIPPING BELOW
            actions.add(new InstantAction(()-> pivot.storage(false, true)));
            actions.add(new SleepAction(0.15));
            actions.add(new InstantAction(()-> pivot.storage(true, false)));
            actions.add(new InstantAction(()-> pivot.arm.storage()));
        } else {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.4));
            actions.add(new InstantAction(() -> pivot.storage(true, false)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        }

        actions.add(new InstantAction(() -> state = BotState.STORAGE));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopStorageOpenGripper() {
        List<Action> actions = new ArrayList<>();

        actions.add(new InstantAction(() -> gripper.open()));

        if (state == BotState.FRONT_INTAKE) {
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
        } else if (state == BotState.WALL_INTAKE) {
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.HIGH_BUCKET) {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.9));
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.LOW_BUCKET) {
            actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.storage(true, true)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        } else if (state == BotState.HIGH_CHAMBER) {
//            actions.add(new InstantAction(() -> pivot.highChamberTransfer(false, true)));
//            actions.add(new SleepAction(0.05));
//            actions.add(new InstantAction(() -> pivot.arm.outtakeHoriz()));
//            actions.add(new InstantAction(() -> pivot.highChamberTransfer(true, false)));
//            actions.add(new SleepAction(0.4));
//            actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
//            actions.add(new InstantAction(() -> pivot.storage(false, true)));
//            actions.add(new SleepAction(0.5));
//            actions.add(new InstantAction(() -> pivot.storage(true, true)));
//            actions.add(new InstantAction(() -> pivot.arm.storage()));

            //for SLIDES CLIPPING BELOW
            actions.add(new InstantAction(()-> pivot.storage(false, true)));
            actions.add(new SleepAction(0.15));
            actions.add(new InstantAction(()-> pivot.storage(true, false)));
            actions.add(new InstantAction(()-> pivot.arm.storage()));
        } else {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.4));
            actions.add(new InstantAction(() -> pivot.storage(true, false)));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
        }

        actions.add(new InstantAction(() -> state = BotState.STORAGE));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopShiftDown() {
        List<Action> actions = new ArrayList<>();

        if (pivot.slides.state == Slides.SlidesState.HIGH) {
            actions.add(new InstantAction(() -> pivot.slides.setState(Slides.SlidesState.HIGH2LOW)));
            actions.add(new InstantAction(() -> pivot.goToResetPosition()));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.neutral()));
            actions.add(new SleepAction(0.3));
            actions.add(new InstantAction(() -> pivot.slides.setPower(-0.1)));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.low()));
            actions.add(new InstantAction(() -> {
                while (pivot.slides.getCurrent() < 4000) {
                    pivot.slides.setPower(-0.5);
                }
            }));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0.3)));
            actions.add(new SleepAction(0.1));
            actions.add(new InstantAction(() -> pivot.slides.setPower(-0.6)));
            actions.add(new SleepAction(0.2));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.resetEncoders()));
            actions.add(new InstantAction(() -> pivot.slides.setState(Slides.SlidesState.LOW)));
        }

        return new SequentialAction(actions);
    }

    public SequentialAction teleopShiftUp() {
        List<Action> actions = new ArrayList<>();

        if (pivot.slides.state == Slides.SlidesState.LOW) {
            actions.add(new InstantAction(() -> pivot.slides.setState(Slides.SlidesState.LOW2HIGH)));
            actions.add(new InstantAction(() -> pivot.goToResetPosition()));
            actions.add(new SleepAction(0.25));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.neutral()));
            actions.add(new SleepAction(0.3));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0.1)));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.high()));
            actions.add(new InstantAction(() -> {
                while (pivot.slides.getCurrent() < 2000) {
                    pivot.slides.setPower(0.2);
                }
            }));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.slides.setPower(-0.3)));
            actions.add(new SleepAction(0.1));
            actions.add(new InstantAction(() -> pivot.slides.setPower(1)));
            actions.add(new SleepAction(0.1));
            actions.add(new InstantAction(() -> pivot.slides.setPower(0)));
            actions.add(new InstantAction(() -> pivot.slides.resetEncoders()));
            actions.add(new InstantAction(() -> pivot.slides.setState(Slides.SlidesState.HIGH)));
        }

        return new SequentialAction(actions);
    }

    public SequentialAction teleopLowBucket() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.HIGH_BUCKET || state == BotState.HIGH_CHAMBER) {
            actions.add(new InstantAction(() -> pivot.arm.vertical()));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.25));
        }

        actions.add(new InstantAction(() -> pivot.lowBucket(true, false)));
        actions.add(new SleepAction(0.3));
        actions.add(new InstantAction(() -> pivot.lowBucket(false, true)));
        actions.add(new SleepAction(0.125));
        actions.add(new InstantAction(() -> pivot.arm.bucket()));
        actions.add(new InstantAction(() -> state = BotState.LOW_BUCKET));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopHighBucket() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.LOW_BUCKET) {
            actions.add(new InstantAction(() -> pivot.arm.vertical()));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new InstantAction(() -> pivot.highBucket(true, false)));
            actions.add(new SleepAction(0.25));
        } else {
            actions.add(new InstantAction(() -> pivot.arm.vertical()));
            actions.add(new InstantAction(() -> pivot.highBucket(true, false)));
            actions.add(new SleepAction(0.3));
        }

        actions.add(new InstantAction(() -> pivot.highBucket(false, true)));
        actions.add(new SleepAction(0.70));
        actions.add(new InstantAction(() -> pivot.arm.bucket()));
        actions.add(new InstantAction(() -> state = BotState.HIGH_BUCKET));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopLowChamber() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.HIGH_CHAMBER) {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.4));
        } else if (state == BotState.HIGH_BUCKET) {
            actions.add(teleopStorage());
            actions.add(new SleepAction(0.6));
        }

        actions.add(new InstantAction(() -> pivot.lowChamber(true, false)));
        actions.add(new SleepAction(0.25));
        actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
        actions.add(new InstantAction(() -> pivot.lowChamber(false, true)));
        actions.add(new InstantAction(() -> state = BotState.LOW_CHAMBER));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopHighChamber() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.LOW_CHAMBER) {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.4));
        } else if (state == BotState.HIGH_BUCKET) {
            actions.add(teleopStorage());
            actions.add(new SleepAction(0.6));
        }

        actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
        actions.add(new InstantAction(() -> pivot.highChamber(true, false)));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.highChamber(false, true)));
        actions.add(new InstantAction(() -> state = BotState.HIGH_CHAMBER));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopPrel2Climb() {
        List<Action> actions = new ArrayList<>();

        if (state != BotState.STORAGE) {
            actions.add(teleopStorage());
            actions.add(new SleepAction(0.5));
        }

        actions.add(new InstantAction(() -> pivot.arm.bucket()));
        actions.add(new InstantAction(() -> pivot.prel2Climb(true, false)));
        actions.add(new SleepAction(0.15));
        actions.add(new InstantAction(() -> pivot.prel2Climb(false, true)));
        actions.add(new InstantAction(() -> state = BotState.CLIMBING));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopClipCancel() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
        return new SequentialAction(actions);
    }

    public SequentialAction teleopClipDown() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(() -> pivot.arm.outtakeDown()));
        return new SequentialAction(actions);
    }

    public SequentialAction teleopClipStorage() {
        List<Action> actions = new ArrayList<>();

        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> gripper.open()));
        actions.add(new SleepAction(0.35));
        actions.add(new InstantAction(() -> pivot.arm.outtakeUp()));
        actions.add(teleopStorageOpenGripper());
        actions.add(new SleepAction(0.3));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopSlidesHighChamber() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.LOW_CHAMBER) {
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.4));
        } else if (state == BotState.HIGH_BUCKET) {
            actions.add(teleopStorage());
            actions.add(new SleepAction(0.6));
        }

        actions.add(new InstantAction(() -> pivot.arm.chamberHoriz()));
        actions.add(new InstantAction(() -> pivot.slidesHighChamber(true, false)));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.slidesHighChamber(false, true)));
        actions.add(new InstantAction(() -> state = BotState.HIGH_CHAMBER));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopSlidesClipCancel() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(() -> pivot.slidesUnClip()));
        return new SequentialAction(actions);
    }

    public SequentialAction teleopSlidesClipDown() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(() -> pivot.slidesClip()));
        actions.add(new SleepAction(0.3));
        return new SequentialAction(actions);
    }

    public SequentialAction teleopSlidesClipStorage() {
        List<Action> actions = new ArrayList<>();

        actions.add(new SleepAction(0.05));
        actions.add(new InstantAction(() -> gripper.open()));
        actions.add(new SleepAction(0.1));
        actions.add(teleopStorageOpenGripper());
        actions.add(new SleepAction(0.3));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopL2Climb() {
        List<Action> actions = new ArrayList<>();
        climbing = true;

        actions.add(new InstantAction(() -> pivot.midl2Climb(false, true)));
        actions.add(new SleepAction(0.15));
        actions.add(new InstantAction(() -> pivot.postl2Climb(true, false)));
        actions.add(new SleepAction(0.175));
        actions.add(new InstantAction(() -> pivot.postl2Climb(false, true)));
        actions.add(new SleepAction(2));
        actions.add(new InstantAction(() -> pivot.climbTransfer(true, false)));
        actions.add(new SleepAction(0.55));
        climbing = false;
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.climbTransfer(false, true)));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopL3Climb() {
        List<Action> actions = new ArrayList<>();
        climbing = true;

        actions.add(new InstantAction(() -> pivot.prel3Climb(true, false)));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.prel3Climb(false, true)));
        actions.add(new SleepAction(1.5));
        actions.add(new InstantAction(() -> pivot.midl3Climb(true, false)));
        actions.add(new SleepAction(0.4));
        actions.add(new InstantAction(() -> pivot.arm.outtakeHoriz()));
        actions.add(new SleepAction(0.15));
        actions.add(new InstantAction(() -> pivot.midl3Climb(false, true)));
        actions.add(new SleepAction(0.35));
        actions.add(new InstantAction(() -> pivot.tiltedl3Climb(true, false)));
        actions.add(new SleepAction(0.8));
        actions.add(new InstantAction(() -> pivot.tiltedl3Climb(false, true)));
        actions.add(new SleepAction(1.7));
        actions.add(new InstantAction(() -> pivot.backTiltedl3Climb(true, false)));
        actions.add(new SleepAction(1.05));
        actions.add(new InstantAction(() -> pivot.postl3Climb(false, true)));
        actions.add(new SleepAction(1));
        actions.add(new InstantAction(() -> pivot.postl3Climb(true, false)));
        climbing = false;
        return new SequentialAction(actions);
    }

    public SequentialAction teleopBucketDrop() {
        List<Action> actions = new ArrayList<>();

        actions.add(new InstantAction(() -> pivot.arm.bucketDrop()));
        actions.add(new InstantAction(() -> gripper.open()));
        actions.add(new SleepAction(0.4));
        actions.add(teleopStorage());

        return new SequentialAction(actions);
    }

    public SequentialAction teleopPickupFrontIntakeToStorage(boolean trigger) {
        List<Action> actions = new ArrayList<>();
        actions.add(teleopPickUpMore());
        actions.add(new SleepAction(0.325));
        if (isHolding() || trigger) {
            actions.add(new SleepAction(0.125));
            actions.add(new InstantAction(() -> pivot.arm.frontPickupToStorage()));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.3));
            actions.add(new InstantAction(() -> pivot.storage(true, false)));
            actions.add(new SleepAction(0.3));
            actions.add(new InstantAction(() -> pivot.arm.storage()));
            actions.add(teleopStorage());
        } else {
            actions.add(teleopOpenGripper());
        }
        return new SequentialAction(actions);
    }

    public SequentialAction teleopPickupFrontIntakeToHighBucket(boolean trigger) {
        List<Action> actions = new ArrayList<>();
        actions.add(teleopPickUpMore());
        actions.add(new SleepAction(0.325));
        if (isHolding() || trigger) {
            actions.add(new SleepAction(0.125));
            actions.add(new InstantAction(() -> pivot.arm.frontPickupToStorage()));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.3));
            actions.add(new InstantAction(() -> pivot.arm.vertical()));
            actions.add(new InstantAction(() -> pivot.highBucket(true, false)));
            actions.add(new SleepAction(0.5));
            actions.add(new InstantAction(() -> pivot.highBucket(false, true)));
            actions.add(new SleepAction(0.70));
            actions.add(new InstantAction(() -> pivot.arm.bucket()));
            actions.add(new InstantAction(() -> state = BotState.HIGH_BUCKET));
        } else {
            actions.add(teleopOpenGripper());
        }
        return new SequentialAction(actions);
    }

    public SequentialAction teleopPickupFrontIntakeToWallIntake(boolean trigger) {
        List<Action> actions = new ArrayList<>();
        actions.add(teleopPickUpMore());
        actions.add(new SleepAction(0.325));
        if (isHolding() || trigger) {
            actions.add(new SleepAction(0.125));
            actions.add(new InstantAction(() -> pivot.arm.frontPickupToStorage()));
            actions.add(new SleepAction(0.075));
            actions.add(new InstantAction(() -> pivot.storage(false, true)));
            actions.add(new SleepAction(0.2));
            actions.add(new InstantAction(() -> pivot.teleopWallIntake(true, false)));
            actions.add(new InstantAction(() -> pivot.arm.vertical()));
            actions.add(new SleepAction(0.1));
            actions.add(new InstantAction(() -> pivot.arm.wallPickup()));
            actions.add(new SleepAction(0.1));
            actions.add(new InstantAction(() -> pivot.teleopWallIntake(false, true)));
            actions.add(new InstantAction(() -> {
                state = BotState.WALL_INTAKE;
            }));
        } else {
            actions.add(teleopOpenGripper());
        }

        return new SequentialAction(actions);
    }

    public SequentialAction teleopFrontIntakeToStorage() {
        List<Action> actions = new ArrayList<>();
        actions.add(teleopPickUp());
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.arm.frontPickupToStorage()));
        actions.add(new SleepAction(0.075));
        actions.add(new InstantAction(() -> pivot.storage(false, true)));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.storage(true, false)));
        actions.add(new SleepAction(0.3));
        actions.add(new InstantAction(() -> pivot.arm.storage()));
        actions.add(teleopStorage());

        return new SequentialAction(actions);
    }

    public SequentialAction teleopFrontIntake() {
        List<Action> actions = new ArrayList<>();

        if (state == BotState.LOW_CHAMBER || state == BotState.HIGH_CHAMBER) {
            actions.add(teleopStorage());
            actions.add(new SleepAction(0.225));
        }
        actions.add(new InstantAction(() -> gripper.open()));
        //actions.add(new InstantAction(() -> pivot.arm.frontPickupToStorage()));
        actions.add(new SleepAction(0.1));
        actions.add(new InstantAction(() -> pivot.frontIntake(true, false)));
        actions.add(new InstantAction(() -> pivot.arm.frontPickup()));
        actions.add(new SleepAction(0.15));
        actions.add(new InstantAction(() -> pivot.frontIntake(false, true)));
        actions.add(new InstantAction(() -> state = BotState.FRONT_INTAKE));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopPickDown() {
        List<Action> actions = new ArrayList<>();

        actions.add(new InstantAction(() -> pivot.changeXZ(+0.5, -pickDownUpValue)));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> gripper.close()));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopUnPick() {
        List<Action> actions = new ArrayList<>();

        actions.add(new InstantAction(() -> gripper.open()));
        actions.add(new SleepAction(0.2));
        actions.add(new InstantAction(() -> pivot.changeZ(pickDownUpValue)));

        return new SequentialAction(actions);
    }

    public SequentialAction teleopPickUp() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(pickDownUpValue))
        );
    }

    public SequentialAction teleopPickUpMore() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(pickDownUpValue + 1))
        );
    }

    public SequentialAction teleopWallIntakeOpen() {
        return new SequentialAction(
                new InstantAction(() -> pivot.teleopWallIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> {
                    pivot.arm.wallPickup();
                    gripper.open();
                    state = BotState.WALL_INTAKE;
                })
        );
    }

    public SequentialAction teleopOpenGripper() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open())
        );
    }

    public SequentialAction teleopCloseGripper() {
        return new SequentialAction(
                new InstantAction(() -> gripper.close())
        );
    }

    public SequentialAction teleopWallIntakeClosed() {
        return new SequentialAction(
                new InstantAction(() -> gripper.close()),
                new InstantAction(() -> pivot.teleopWallIntake(true, true)),
                new SleepAction(0.2),
                new InstantAction(() -> {
                    pivot.arm.wallPickup();
                    state = BotState.WALL_INTAKE;
                })
        );
    }

    public void goToReset() {
        pivot.goToResetPosition();
        state = BotState.RESETTING;
    }

    public void resetSlides() {
        pivot.slides.resetEncoders();
    }

    public void initialize() {
        storage();
        pivot.slides.high();
    }

    public void pivotStorage() {
        pivot.storage(true, true);
        state = BotState.STORAGE;
    }


    //FOR AUTONOMOUS


    public void subAutoPickUp() {
        pivot.changeZ(6);
    }

    public void pickUp() {
        pivot.changeZ(pickDownUpValue);
    }

    public SequentialAction actionSubAutoPickDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.pitchPickup()),
                new InstantAction(() -> pivot.changeXZ(pipeline.getSlidesY(), -13, true, true)),//7.1
                new SleepAction(0.55),
                new InstantAction(() -> gripper.close()),
                new SleepAction(0.1)
        );
    }

    public SequentialAction actionSubAutoPickUp() {
        return new SequentialAction(
                new InstantAction(this::subAutoPickUp),
                new SleepAction(0.1)
        );
    }

    public SequentialAction actionSubAutoIntake() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.frontPickupToStorage()),
                new InstantAction(() -> pivot.frontIntakeStorage(true, false)),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.subAutoIntake(true, true)),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.arm.cv())
        );
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
                new SleepAction(0.25),
                new InstantAction(() -> pivot.highChamber(false, true)),
                new SleepAction(0.05),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> state = BotState.HIGH_CHAMBER)
        );
    }

    public SequentialAction actionWallToHighChamber() {
        return new SequentialAction(
                new InstantAction(() -> pivot.highChamber(true, false)),
                new SleepAction(0.8),
                new InstantAction(() -> pivot.highChamber(false, true)),
                new SleepAction(0.05),
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> state = BotState.HIGH_CHAMBER)
        );
    }

    public SequentialAction actionWallToSlidesHighChamber() {
        return new SequentialAction(
                new InstantAction(() -> pivot.autoSlidesHighChamber(true, false)),
                new SleepAction(0.4),
                new InstantAction(() -> pivot.autoSlidesHighChamber(false, true)),
                new InstantAction(() -> pivot.arm.chamberHoriz()),
                new InstantAction(() -> state = BotState.HIGH_CHAMBER)
        );
    }

    public SequentialAction actionSlidesClipWall() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.arm.vertical()),
                new SleepAction(0.05),
                new InstantAction(() -> pivot.autoWallIntake(false, true)),
                new InstantAction(() -> pivot.arm.wallPickup()),
                new SleepAction(0.1),
                new InstantAction(() -> pivot.autoWallIntake(true, false)),
                new InstantAction(() -> state = BotState.WALL_INTAKE)
        );
    }

    public SequentialAction actionSlidesClipDown() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(() -> pivot.slidesClip()));
        actions.add(new SleepAction(0.4));
        actions.add(new InstantAction(() -> gripper.open()));
        actions.add(new SleepAction(0.05));
        return new SequentialAction(actions);
    }

    public SequentialAction actionSlidesClipStorage() {
        List<Action> actions = new ArrayList<>();
        actions.add(new InstantAction(()-> pivot.storage(false, true)));
        actions.add(new InstantAction(()-> pivot.highChamberTransfer(true, false)));
        actions.add(new SleepAction(0.15));
        actions.add(new InstantAction(()-> pivot.storage(true, false)));
        actions.add(new InstantAction(()-> pivot.arm.storage()));
        return new SequentialAction(actions);
    }

    public SequentialAction actionPickUp() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(5))
        );
    }

    public SequentialAction actionPickDown() {
        return new SequentialAction(
                new InstantAction(() -> pivot.changeZ(-5.2)),
                new SleepAction(0.25),
                new InstantAction(() -> gripper.close()),
                new SleepAction(0.125)
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
                new InstantAction(() -> pivot.arm.bucketDrop()),
                new SleepAction(0.15),
                new InstantAction(() -> gripper.open()),
                new SleepAction(0.12),
                new InstantAction(() -> checkBreakBeam(false)),
                new InstantAction(() -> pivot.arm.outtakeDown())
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
                new InstantAction(() -> pivot.storage(false, true)),
                new SleepAction(0.425),
                new InstantAction(() -> pivot.highBucket(false, true)),
                new InstantAction(() -> pivot.arm.outtakeDown()),
                new SleepAction(0.70),
                new InstantAction(() -> pivot.arm.bucket()),
                new InstantAction(() -> state = BotState.HIGH_BUCKET)
        );
    }

    public SequentialAction actionBucketToFrontIntake() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeDown()),
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.frontAutoIntake(true, false)),
//                new SleepAction(0.3),
//                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
//                new SleepAction(0.075),
                new InstantAction(() -> state = BotState.FRONT_INTAKE)
        );
    }

    public SequentialAction actionBucketToCVPosition() {
        return new SequentialAction(
                new InstantAction(() -> pivot.arm.outtakeUp()),
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.storage(false, true)),
                new InstantAction(() -> pivot.arm.frontPickup()),
                new SleepAction(0.2),
                new InstantAction(() -> pivot.subAutoIntake(true, true)),
                new SleepAction(0.3),
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
                new InstantAction(() -> pivot.frontAutoIntake(true, false)),
                new SleepAction(0.55),
                new InstantAction(() -> pivot.frontAutoIntake(false, true)),
                new SleepAction(0.1),
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

    public SequentialAction actionMoreWallIntakeOpen() {
        return new SequentialAction(
                new InstantAction(() -> gripper.open()),
                new InstantAction(() -> pivot.autoMoreWallIntake(true, true)),
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
            autoPeriodic();
            return true;
        }
    }

    public void periodic() {
        pivot.periodic(fr.getCurrentPosition());
    }

    public void autoPeriodic() {
        pivot.periodic(-fr.getCurrentPosition());
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
        double frontWheelModifier = (state == BotState.FRONT_INTAKE)? 1.2 : 1.02;
        if (state == BotState.HIGH_CHAMBER) frontWheelModifier = 1.06;
        double rearWheelModifier = (state == BotState.HIGH_BUCKET || state == BotState.LOW_BUCKET)? 1.07 : 1;
        if (state == BotState.WALL_INTAKE) rearWheelModifier = 1.0;
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

    public static double vectorDiff(Vector2d v1, Vector2d v2) {
        double dx = v1.x - v2.x;
        double dy = v1.y - v2.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

//    public void subAutoIntake() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.LOW_CHAMBER | state == BotState.HIGH_CHAMBER) {
//                    storage();
//                    Thread.sleep(225);
//                }
//                gripper.open();
//                pivot.arm.frontPickupToStorage();
//                Thread.sleep(100);
//                pivot.frontIntakeStorage(true, false);
//                Thread.sleep(100);
//                pivot.subAutoIntake(true, true);
//                Thread.sleep(300);
//                pivot.arm.frontPickup();
//                state = BotState.FRONT_INTAKE;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
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
                } else if (state == BotState.LOW_BUCKET) {
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
//
//    public void storageOpenGripper() {
//        Thread thread = new Thread(() -> {
//            try {
//                gripper.open();
//                if (state == BotState.FRONT_INTAKE) {
//                    pivot.storage(true, true);
//                } else if (state == BotState.WALL_INTAKE) {
//                    pivot.storage(true, true);
//                    Thread.sleep(250);
//                    pivot.arm.storage();
//                } else if (state == BotState.HIGH_BUCKET) {
//                    pivot.storage(false, true);
//                    Thread.sleep(900);
//                    pivot.storage(true, true);
//                    pivot.arm.storage();
//                } else if (state == BotState.HIGH_CHAMBER){
//                    pivot.highChamberTransfer(false, true);
//                    Thread.sleep(50);
//                    pivot.arm.outtakeHoriz();
//                    pivot.highChamberTransfer(true, false);
//                    Thread.sleep(400);
//                    pivot.arm.outtakeUp();
//                    pivot.storage(false, true);
//                    Thread.sleep(500);
//                    pivot.storage(true, true);
//                    pivot.arm.storage();
//                } else {
//                    pivot.storage(false, true);
//                    Thread.sleep(400);
//                    pivot.storage(true, false);
//                    pivot.arm.storage();
//                }
//                state = BotState.STORAGE;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void shiftDown() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.slides.setState(Slides.SlidesState.HIGH2LOW);
//                pivot.goToResetPosition();
//                Thread.sleep(250);
//                pivot.slides.setPower(0);
//                pivot.slides.neutral();
//                Thread.sleep(300);
//                pivot.slides.setPower(-0.1); //Upwards (for high) to help it disengage
//                Thread.sleep(75);
//                pivot.slides.setPower(0);
//                pivot.slides.low(); //Engage high
//                while (pivot.slides.getCurrent() < 4000) {
//                    pivot.slides.setPower(-0.5); //Downwards (for low) to help it engage
//                }
//                Thread.sleep(75);
//                pivot.slides.setPower(0.3);
//                Thread.sleep(100);
//                pivot.slides.setPower(-0.6); //run it into the ground
//                Thread.sleep(200);
//                pivot.slides.setPower(0);
//                pivot.slides.resetEncoders();
//                pivot.slides.setState(Slides.SlidesState.LOW);
//            } catch (InterruptedException ignored) {}
//        });
//        if (pivot.slides.state == Slides.SlidesState.HIGH)
//            thread.start();
//    }
//
//    public void shiftUp() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.slides.setState(Slides.SlidesState.LOW2HIGH);
//                pivot.goToResetPosition();
//                Thread.sleep(250);
//                pivot.slides.setPower(0);
//                pivot.slides.neutral();
//                Thread.sleep(300);
//                pivot.slides.setPower(0.1); //Upwards (for low) to help it disengage
//                Thread.sleep(75);
//                pivot.slides.setPower(0);
//                pivot.slides.high(); //Engage high
//                while (pivot.slides.getCurrent() < 2000) {
//                    pivot.slides.setPower(0.2); //Downwards (for high) to help it engage
//                }
//                Thread.sleep(75);
//                pivot.slides.setPower(-0.3);
//                Thread.sleep(100);
//                pivot.slides.setPower(1); //run it into the ground
//                Thread.sleep(100);
//                pivot.slides.setPower(0);
//                pivot.slides.resetEncoders();
//                pivot.slides.setState(Slides.SlidesState.HIGH);
//            } catch (InterruptedException ignored) {}
//        });
//        if (pivot.slides.state == Slides.SlidesState.LOW)
//            thread.start();
//    }
//
//    public void lowBucket() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.HIGH_BUCKET || state == BotState.HIGH_CHAMBER) {
//                    pivot.arm.vertical();
//                    pivot.storage(false, true); //pull slides in so that it doesn't hit
//                    Thread.sleep(250);
//                }
//                pivot.lowBucket(true, false);
//                Thread.sleep(300);
//                pivot.lowBucket(false, true);
//                Thread.sleep(125);
//                pivot.arm.bucket();
//                state = BotState.LOW_BUCKET;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void highBucket() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.LOW_BUCKET) { //TODO: Test if hits bucket
//                    pivot.arm.vertical();
//                    pivot.storage(false, true); //pull slides in so that it doesn't hit
//                    pivot.highBucket(true, false);
//                    Thread.sleep(250);
//                } else {
//                    pivot.highBucket(true, false);
//                    Thread.sleep(300);
//                }
//                pivot.highBucket(false, true);
//                Thread.sleep(650);
//                pivot.arm.bucket();
//                state = BotState.HIGH_BUCKET;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void lowChamber() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.HIGH_CHAMBER) { //TODO: Test if hits chamber rod
//                    pivot.storage(false, true); //pull slides in so that it doesn't hit
//                    Thread.sleep(400);
//                } else if (state == BotState.HIGH_BUCKET) {
//                    storage();
//                    Thread.sleep(600);
//                }
//                pivot.lowChamber(true, false);
//                Thread.sleep(250);
//                pivot.arm.outtakeUp();
//                pivot.lowChamber(false, true);
//                state = BotState.LOW_CHAMBER;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void highChamber() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.LOW_CHAMBER) { //TODO: Test if hits chamber rod
//                    pivot.storage(false, true); //pull slides in so that it doesn't hit
//                    Thread.sleep(400);
//                } else if (state == BotState.HIGH_BUCKET) {
//                    storage();
//                    Thread.sleep(600);
//                }
//                pivot.arm.outtakeUp();
//                pivot.highChamber(true, false);
//                Thread.sleep(200);
//                pivot.highChamber(false, true);
//                state = BotState.HIGH_CHAMBER;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void prel2Climb() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state != BotState.STORAGE) { //TODO: Test if hits chamber rod
//                    storage();
//                    Thread.sleep(500);
//                }
//                pivot.arm.bucket();
//                pivot.prel2Climb(true, false);
//                Thread.sleep(150);
//                pivot.prel2Climb(false, true);
//                state = BotState.CLIMBING;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void l2Climb() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.midl2Climb(false, true);
//                Thread.sleep(150);
//                pivot.postl2Climb(true, false);
//                Thread.sleep(175);
//                pivot.postl2Climb(false, true);
//                Thread.sleep(2150);
//                pivot.climbTransfer(true, false);
//                Thread.sleep(750);
//                pivot.climbTransfer(false, true);
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void l3Climb() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.prel3Climb(true, false);
//                Thread.sleep(200);
//                pivot.prel3Climb(false, true);
//                Thread.sleep(1500);
//                pivot.midl3Climb(true, false);
//                Thread.sleep(400);
//                pivot.arm.outtakeHoriz();
//                Thread.sleep(150);
//                pivot.midl3Climb(false, true);
//                Thread.sleep(250);
//                pivot.tiltedl3Climb(true, false);
//                Thread.sleep(800);
//                pivot.tiltedl3Climb(false, true);
//                Thread.sleep(1700);
//                pivot.backTiltedl3Climb(true, false);
//                Thread.sleep(500);
//                pivot.postl3Climb(false, true);
//                Thread.sleep(1000);
//                pivot.postl3Climb(true, false);
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void clipCancel() {
//        pivot.arm.outtakeUp();
//    }
//
//    public void clipDown() {
//        pivot.arm.outtakeDown();
//    }
//
//    public void clipStorage() {
//        Thread thread = new Thread(() -> {
//            try {
//                Thread.sleep(200);
//                gripper.open();
//                Thread.sleep(350);
//                pivot.arm.outtakeUp();
//                storageOpenGripper();
//                Thread.sleep(300);
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void bucketDrop() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.arm.bucketDrop();
//                gripper.open();
//                Thread.sleep(400);
//                storage();
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void frontIntakeToStorage() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.changeZ(3.5);
//                Thread.sleep(125);
//                pivot.arm.outtakeHoriz();
//                Thread.sleep(75);
//                pivot.storage(false, true);
//                Thread.sleep(300);
//                pivot.storage(true, false);
//                Thread.sleep(300);
//                pivot.arm.storage();
//                storage();
//
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void frontIntake() {
//        Thread thread = new Thread(() -> {
//            try {
//                if (state == BotState.LOW_CHAMBER | state == BotState.HIGH_CHAMBER) {
//                    storage();
//                    Thread.sleep(225);
//                }
//                gripper.open();
//                pivot.arm.frontPickupToStorage();
//                Thread.sleep(100);
//                pivot.frontIntakeStorage(true, false);
//                Thread.sleep(100);
//                pivot.frontIntake(true, true);
//                Thread.sleep(300);
//                pivot.arm.frontPickup();
//                state = BotState.FRONT_INTAKE;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void pickDown() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.changeZ(-pickDownUpValue);
//                Thread.sleep(200);
//                gripper.close();
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void unPick() {
//        Thread thread = new Thread(() -> {
//            try {
//                gripper.open();
//                Thread.sleep(200);
//                pivot.changeZ(pickDownUpValue);
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void subAutoPickDown() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.changeZ(-5.5);
//                Thread.sleep(500);
//                pivot.changeZ(-4.5);
//                Thread.sleep(400);
//                gripper.close();
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void subAutoPickUp() {
//        pivot.changeZ(6);
//    }
//
//    public void pickUp() {
//        pivot.changeZ(pickDownUpValue);
//    }
//
//    public void wallIntakeOpen() {
//        Thread thread = new Thread(() -> {
//            try {
//                pivot.teleopWallIntake(true, true);
//                Thread.sleep(200);
//                pivot.arm.wallPickup();
//                gripper.open();
//                state = BotState.WALL_INTAKE;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }
//
//    public void openGripper() {
//        gripper.open();
//    }
//
//    public void wallIntakeClosed() {
//        Thread thread = new Thread(() -> {
//            try {
//                gripper.close();
//                pivot.teleopWallIntake(true, true);
//                Thread.sleep(200);
//                pivot.arm.wallPickup();
//                state = BotState.WALL_INTAKE;
//            } catch (InterruptedException ignored) {}
//        });
//        thread.start();
//    }

    }