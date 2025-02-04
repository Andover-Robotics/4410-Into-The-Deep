package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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



            //STORAGE
            if (bot.state == Bot.BotState.STORAGE) {
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                    bot.teleopGoToReset();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    runningActions.add(bot.teleopShiftUp());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    runningActions.add(bot.teleopShiftDown());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(bot.teleopStorage());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    runningActions.add(bot.teleopPrel2Climb());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopFrontIntake());
                    intakeCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3) {
                        runningActions.add(bot.teleopWallIntakeClosed());
                    } else {
                        runningActions.add(bot.teleopWallIntakeOpen());
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    runningActions.add(bot.teleopLowChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    runningActions.add(bot.teleopHighChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.add(bot.teleopHighBucket());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.add(bot.teleopLowBucket());
                }
            }

            if (bot.state == Bot.BotState.CLIMBING) {
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

                bot.pivot.runManualIK(gp2.getLeftY() / 1.15);
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    runningActions.add(bot.teleopPickDown());
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.X) && !intakeCancel) {
                    runningActions.add(bot.teleopPickupFrontIntakeToStorage(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25));
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
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    // runningActions.add(bot.teleopClipDown()); // Uncomment if needed
                }
                if (gp2.wasJustReleased(GamepadKeys.Button.Y) && !clipCancel) {
                    runningActions.add(bot.teleopClipStorage());
                } else if (gp2.wasJustReleased(GamepadKeys.Button.Y) && clipCancel) {
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B) && gp2.isDown(GamepadKeys.Button.Y)) {
                    clipCancel = true;
                    runningActions.add(bot.teleopClipCancel());
                }
                if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                    runningActions.add(bot.teleopOpenGripper());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    runningActions.add(bot.teleopLowChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    runningActions.add(bot.teleopHighChamber());
                    clipCancel = false;
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
                    runningActions.add(bot.teleopLowChamber());
                    clipCancel = false;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    runningActions.add(bot.teleopHighChamber());
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
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            // DRIVE
            drive();

            // TELEMETRY
            telemetry.addData("Bot State", bot.state);
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot IK Target Angle", bot.pivot.pivotIKTargetDegrees);
            telemetry.addData("Pivot Motor Current", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Pivot PID", bot.pivot.power - bot.pivot.calculateFeedForward());
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
            telemetry.addData("Target X", bot.pivot.targetX);
            telemetry.addData("Target Z", bot.pivot.targetZ);
            telemetry.addData("Manual IK Piv/Sli", bot.pivot.manualIK);
            telemetry.addData( "Slides Manual Power", bot.pivot.slides.manualPower);
            telemetry.addData("\n\nHolding Sample:", bot.getBreakBeam());
            telemetry.addData("\n\ndt current", bot.getMotorCurrent());
            telemetry.update();
            bot.periodic();
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