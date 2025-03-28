package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    public int clipCounter = 0;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric, intakeCancel, clipCancel;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
        bot.stopMotors();
        bot.state = Bot.BotState.STORAGE;
        //bot.storage();

        waitForStart();

        bot.initialize();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

//            if (bot.autoDrive != null) {
//                bot.autoDrive.updatePoseEstimate();
//            }
//
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.initializeAutoClipping();
                clipCounter = 0;
            }
//            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
//                bot.cycleClip();
//            }
//            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
//                bot.rebuildMotors();
//            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                runningActions.add(bot.actionRearClipStorage());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                runningActions.add(bot.actionFrontWallIntake());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                runningActions.add(bot.actionFrontWallToRearSlidesChamber());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                runningActions.add(bot.actionOpenGripper());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.add(bot.actionCloseGripper());
            }



            //STORAGE
            if (bot.state == Bot.BotState.STORAGE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                    bot.teleopGoToReset();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//                    runningActions.add(bot.teleopShiftUp());
                    bot.shiftUp();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                    runningActions.add(bot.teleopShiftDown());
                    bot.shiftDown();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.teleopStorage());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    runningActions.add(bot.teleopPrel2Climb());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopFrontIntake(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
                        runningActions.add(bot.teleopWallIntakeClosed());
                    } else {
                        runningActions.add(bot.teleopWallIntakeOpen());
                    }
                }
//                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                    runningActions.add(bot.teleopLowChamber());
//                    clipCancel = false;
//                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    runningActions.add(bot.teleopSlidesHighChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.add(bot.teleopHighBucket());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.add(bot.teleopLowBucket());
                }
            }

            if (bot.state == Bot.BotState.CLIMBING && !bot.climbing) {
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    runningActions.add(bot.teleopL2Climb());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    runningActions.add(bot.teleopL3Climb());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.teleopStorage());
                }
            }

            if (bot.state == Bot.BotState.FRONT_INTAKE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && !gp2.isDown(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopFrontIntakeToStorage());
                }
                bot.pivot.runManualIK(gp2.getLeftY() / 1.01);
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopPickDown());
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.X) && !intakeCancel) {
                    if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
                        runningActions.add(bot.teleopPickupFrontIntakeToWallIntake(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25));
                    } else if (gp2.isDown(GamepadKeys.Button.DPAD_UP)){
                        runningActions.add(bot.teleopPickupFrontIntakeToHighBucket(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25));
                    } else {
                        runningActions.add(bot.teleopPickupFrontIntakeToStorage(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25));
                    }
                    } else if (gp2.wasJustReleased(GamepadKeys.Button.X) && intakeCancel) {
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && gp2.isDown(GamepadKeys.Button.X)) {
                    intakeCancel = true;
                    runningActions.add(bot.teleopPickUp());
                    runningActions.add(bot.teleopOpenGripper());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    runningActions.add(new InstantAction(() -> bot.pivot.arm.rollLeft()));
                } else if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    runningActions.add(new InstantAction(() -> bot.pivot.arm.rollRight()));
                }
            }

            if (bot.state == Bot.BotState.WALL_INTAKE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && !gp2.isDown(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopStorage());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopCloseGripper());
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.X) && !intakeCancel) {
                    runningActions.add(bot.teleopStorage());
                } else if (gp2.wasJustReleased(GamepadKeys.Button.X) && intakeCancel) {
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A) && gp2.isDown(GamepadKeys.Button.X)) {
                    intakeCancel = true;
                    runningActions.add(bot.teleopOpenGripper());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    runningActions.add(bot.teleopOpenGripper());
                }
            }

            if (bot.state == Bot.BotState.HIGH_CHAMBER || bot.state == Bot.BotState.LOW_CHAMBER) {
                bot.pivot.runManualIK(gp2.getLeftY() / 1.15);
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.teleopStorage());
                }
//                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
//                    // runningActions.add(bot.teleopClipDown()); // Uncomment if needed
//                }
//                if (gp2.wasJustReleased(GamepadKeys.Button.Y) && !clipCancel) {
//                    runningActions.add(bot.teleopClipStorage());
//                } else if (gp2.wasJustReleased(GamepadKeys.Button.Y) && clipCancel) {
//                    clipCancel = false;
//                }
//                if (gp2.wasJustPressed(GamepadKeys.Button.B) && gp2.isDown(GamepadKeys.Button.Y)) {
//                    clipCancel = true;
//                    runningActions.add(bot.teleopClipCancel());
//                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    runningActions.add(bot.teleopSlidesClipDown());
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.Y) && !clipCancel) {
                    runningActions.add(bot.teleopSlidesClipStorage());
                } else if (gp2.wasJustReleased(GamepadKeys.Button.Y) && clipCancel) {
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B) && gp2.isDown(GamepadKeys.Button.Y)) {
                    clipCancel = true;
                    runningActions.add(bot.teleopSlidesClipCancel());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    runningActions.add(bot.teleopOpenGripper());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.add(bot.teleopHighBucket());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.add(bot.teleopLowBucket());
                }
            }

            if (bot.state == Bot.BotState.HIGH_BUCKET || bot.state == Bot.BotState.LOW_BUCKET) {
                bot.pivot.runManualIK(gp2.getLeftY());

                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.teleopStorage());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    runningActions.add(bot.teleopBucketDrop());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                    runningActions.add(bot.teleopLowChamber());
//                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    runningActions.add(bot.teleopSlidesHighChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.add(bot.teleopHighBucket());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.add(bot.teleopLowBucket());
                }
            }

            if (bot.state == Bot.BotState.RESETTING) {
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                    bot.resetSlides();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(new InstantAction(() -> bot.teleopPivotStorage()));
                }
            }

            //Execute Actions
            if (!bot.autonomous){
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;
            } else {
                Actions.runBlocking(
                        new ActionHelpersJava.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
                                        bot.autoDrive.actionBuilder(bot.clipIntake)
                                                .stopAndAdd(new SequentialAction(
                                                        bot.actionCloseGripper(),
                                                        new SleepAction(0.15)
                                                ))

                                                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                                                .strafeToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(bot.chamber.component1().x - ((double) clipCounter), bot.chamber.component1().y), bot.autoDrive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                                                .stopAndAdd(new SequentialAction(
                                                        bot.actionRearSlidesClipDown(),
                                                        new SleepAction(0.35),
                                                        bot.actionOpenGripper()
                                                ))

                                                .afterTime(0.01, bot.actionRearClipWall())

                                                .strafeToLinearHeading(bot.clipIntake.component1(), Math.toRadians(90), bot.autoDrive.defaultVelConstraint, new ProfileAccelConstraint(-45, 90))
//                                                .setTangent(Math.toRadians(90))
//                                                .splineToConstantHeading(bot.clipIntake.component1(), Math.toRadians(110), bot.autoDrive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))

//                                                .stopAndAdd(new SequentialAction(
//                                                        bot.actionCloseGripper(),
//                                                        new SleepAction(0.15)
//                                                ))
//                                                .afterTime(0.05, bot.actionFrontWallToRearSlidesChamber())
//                                                .strafeToConstantHeading(bot.chamber.component1())
//                                                .stopAndAdd(new SequentialAction(
//                                                        bot.actionRearSlidesClipDown(),
//                                                        new SleepAction(0.35),
//                                                        bot.actionOpenGripper()
//                                                ))
//
//                                                .afterTime(0.01, bot.actionRearClipWall())
//
//                                                .strafeToConstantHeading(new com.acmerobotics.roadrunner.Vector2d(bot.clipIntake.component1().x, bot.clipIntake.component1().y-10))
//                                                .strafeToConstantHeading(bot.clipIntake.component1(), bot.autoDrive.defaultVelConstraint, new ProfileAccelConstraint(-35, 55))

                                                .build()
                                ),
                                bot.checkAutoClipping(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                        )
                );
                clipCounter++;
//                );
            }

            // DRIVE
            if (!bot.autonomous) {
                drive();
                bot.periodic();
            }

            // TELEMETRY
            if (bot.autoDrive != null) {
                telemetry.addData("bot position", bot.autoDrive.pose);
//                if (bot.controller.currentPoseVel != null) {
//                    telemetry.addData("p2p power", bot.controller.currentPoseVel);
//                } else {
//                    telemetry.addLine("NO POSITION");
//                }
            } else {
                telemetry.addLine("NO POSITION");
                telemetry.addLine("NO POSITION");
            }

            telemetry.addData("Bot State", bot.state);
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot IK Target Angle", bot.pivot.pivotIKTargetDegrees);
            telemetry.addData("Pivot Motor Current", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pivot PID", bot.pivot.power - bot.pivot.calculateFeedForward());
            telemetry.addData("Pivot FF", bot.pivot.calculateFeedForward());
            //telemetry.addData("Pivot Manual Power Up", bot.pivot.manualPowerUp);
            //telemetry.addData("Pivot Target (ticks)", bot.pivot.getTarget());
            //telemetry.addData("Pivot Profiler", bot.pivot.getProfilerTarget());
            telemetry.addData("Pivot Power", bot.pivot.power);

            telemetry.addData("Slides Position (mm)", bot.pivot.slides.getmmPosition());
            telemetry.addData("Slides Position (ticks)", bot.pivot.slides.getPosition());
            telemetry.addData("Slides Target (ticks)", bot.pivot.slides.getControllerSetpoint());
            telemetry.addData("Slides Target (mm)", (bot.pivot.slides.getTargetMM()));
            telemetry.addData("Slides Current", bot.pivot.slides.getCurrent());
            telemetry.addData("Slides Power", bot.pivot.slides.power);
            telemetry.addData("Going Down", bot.pivot.slides.goingDown);
            telemetry.addData("Target X", bot.pivot.targetX);
            telemetry.addData("Target Z", bot.pivot.targetZ);
            telemetry.addData("Manual IK Piv/Sli", bot.pivot.manualIK);
            telemetry.addData( "Slides Manual Power", bot.pivot.slides.manualPower);
            telemetry.addData("\n\nHolding Sample:", bot.isHolding());
            telemetry.addData("\nBreak Beams?", bot.getBreakBeam());
            telemetry.update();
        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1
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