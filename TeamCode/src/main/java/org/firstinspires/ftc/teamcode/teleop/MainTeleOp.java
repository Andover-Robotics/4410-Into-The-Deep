package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.vision.SampleTrackingPipeline;

import java.lang.*;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric, intakeCancel, clipCancel;
    private Thread thread;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleTrackingPipeline sampleTracker = new SampleTrackingPipeline();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), sampleTracker);

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
        //bot.stopMotors();
        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        // left trigger for red, right trigger for blue alliance
        if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 ) {
            sampleTracker.setRedAlliance();
        }

        if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 || gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 ) {
            sampleTracker.setBlueAlliance();
        }



        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            gp1.readButtons();
            gp2.readButtons();

            //STORAGE
            if (bot.state == Bot.BotState.STORAGE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    bot.rearIntake();
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.frontIntake();
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.wallIntake();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.lowChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.highChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.highBucket();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.lowBucket();
                }
            }
            if (bot.state == Bot.BotState.FRONT_INTAKE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && !gp2.isDown(GamepadKeys.Button.X)) {
                    bot.frontIntakeToStorage();
                }
                if (Math.abs(gp2.getLeftY()) > 0.05) {
                    bot.pivot.runManualIK(gp2.getLeftY());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.pickDown();
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.X) && !intakeCancel) {
                    bot.pickUp();
                    sleep(350);
                    bot.frontIntakeToStorage();
                } else if (gp2.wasJustReleased(GamepadKeys.Button.X) && intakeCancel) {
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && gp2.isDown(GamepadKeys.Button.X)) {
                    intakeCancel = true;
                    bot.pickUp();
                }

                if (sampleTracker.alliance == SampleTrackingPipeline.Alliance.RED) {
                    if (sampleTracker.detectYellow) {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleYellow());
                    } else {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleRed());
                    }
                } else {
                    if (sampleTracker.detectYellow) {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleYellow());
                    } else {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleBlue());
                    }

                }

                if (gamepad2.right_trigger > 0.5) {
                    sampleTracker.detectYellow(false);
                } else {
                    sampleTracker.detectYellow(true);
                }
            }
            if (bot.state == Bot.BotState.REAR_INTAKE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && !gp2.isDown(GamepadKeys.Button.X)) {
                    bot.storage();
                }
                if (Math.abs(gp2.getLeftY()) > 0.05) {
                    bot.pivot.runManualIK(gp2.getLeftY());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.pickDown();
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.X) && !intakeCancel) {
                    bot.pickUp();
                    sleep(300);
                    bot.storage();
                } else if (gp2.wasJustReleased(GamepadKeys.Button.X) && intakeCancel) {
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && gp2.isDown(GamepadKeys.Button.X)) {
                    intakeCancel = true;
                    bot.pickUp();
                }
            }
            if (bot.state == Bot.BotState.WALL_INTAKE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.storage();
                }
                if (Math.abs(gp2.getLeftY()) > 0.05) {
                    bot.pivot.runManualIK(gp2.getLeftY());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.gripper.close();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.gripper.open();
                }

                if (sampleTracker.alliance == SampleTrackingPipeline.Alliance.RED) {
                    if (sampleTracker.detectYellow) {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleYellow());
                    } else {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleRed());
                    }
                } else {
                    if (sampleTracker.detectYellow) {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleYellow());
                    } else {
                        bot.pivot.arm.setRoll(sampleTracker.getAngleBlue());
                    }

                }

                if (gamepad2.right_trigger > 0.5) {
                    sampleTracker.detectYellow(false);
                } else {
                    sampleTracker.detectYellow(true);
                }
            }
            if (bot.state == Bot.BotState.HIGH_CHAMBER || bot.state == Bot.BotState.LOW_CHAMBER) {
                if (Math.abs(gp2.getLeftY()) > 0.05) {
                    bot.pivot.runManualIK(gp2.getLeftY());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.clipDown();
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.Y) && !clipCancel) {
                    bot.clipStorage();
                } else if (gp2.wasJustReleased(GamepadKeys.Button.Y) && clipCancel) {
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B) && gp2.isDown(GamepadKeys.Button.Y)) {
                    clipCancel = true;
                    bot.clipCancel();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.lowChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.highChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.highBucket();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.lowBucket();
                }
            }
            if (bot.state == Bot.BotState.HIGH_BUCKET || bot.state == Bot.BotState.LOW_BUCKET) {
//                if (Math.abs(gp2.getLeftY()) > 0.05) {
//                    bot.pivot.runManualIK(gp2.getLeftY());
//                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.bucketDrop();
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.lowChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.highChamber();
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.highBucket();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.lowBucket();
                }
            }



//
//            if (gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
//                bot.gripper.intake();
//            } else if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
//                bot.gripper.outtake();
//            } else {
//                bot.gripper.stop();
//            }

            // DRIVE
            drive();

            // TELEMETRY
            telemetry.addData("Bot State", bot.state);
            telemetry.addData("Pivot Active", bot.pivot.testing);
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot IK Target Angle", bot.pivot.pivotIKTargetDegrees);
            //telemetry.addData("Pivot Motor Current", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Pivot PID", bot.pivot.power - bot.pivot.calculateFeedForward());
            //telemetry.addData("Pivot Manual Power Up", bot.pivot.manualPowerUp);
            //telemetry.addData("Pivot Target (ticks)", bot.pivot.getTarget());
            //telemetry.addData("Pivot Profiler", bot.pivot.getProfilerTarget());
            telemetry.addData("Pivot Power", bot.pivot.power);
            telemetry.addData("Slides Position (mm)", bot.pivot.slides.getmmPosition());
            telemetry.addData("Slides IK Position (mm)", bot.pivot.slides.getIKmmPosition());
            //telemetry.addData("Slides Target (ticks)", bot.pivot.slides.getTarget());
            //telemetry.addData("Slides Profiler", bot.pivot.slides.getProfilerTarget());
            telemetry.addData("Slides Power", bot.pivot.slides.power);
            telemetry.addData("Target X", bot.pivot.targetX);
            telemetry.addData("Target Z", bot.pivot.targetZ);
            telemetry.addData("Current Pitch (Servo)", bot.pivot.arm.currentPitch);
            telemetry.addData("Pitch Setpoint", bot.pivot.arm.pitchSetpoint);
            telemetry.addData("Arm Passive IK", bot.pivot.armTesting);
            telemetry.update();
            bot.pivot.periodic();
        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1, 1/2 when distance sensor
        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        if (fieldCentric) {
            Vector2d driveVector = new Vector2d(-gp1.getLeftX(), -gp1.getLeftY()),
                    turnVector = new Vector2d(-gp1.getRightX(), 0);
            bot.driveFieldCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        } else {
            Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                    turnVector = new Vector2d(gp1.getRightX(), 0);
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }

}