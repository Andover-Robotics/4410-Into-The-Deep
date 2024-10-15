package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import java.lang.*;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric = false;
    private Thread thread;


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
        bot.pivot.arm.setRollPitch(bot.pivot.arm.ROLL_MID, bot.pivot.arm.pitchOuttakeDown);

        double pivotAngle = 90;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            gp1.readButtons();
            gp2.readButtons();

            if (gp2.wasJustPressed(GamepadKeys.Button.START)) {
                bot.pivot.arm.setRollPitch(bot.pivot.arm.ROLL_MID, bot.pivot.arm.pitchOuttakeDown);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                bot.pivot.arm.setRoll(bot.pivot.arm.ROLL_MAX);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.pivot.arm.setRoll(bot.pivot.arm.ROLL_MIN);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                bot.pivot.arm.setPitch(bot.pivot.arm.pitchPickup);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.pivot.arm.setPitch(bot.pivot.arm.pitchOuttakeDown);
            }

            bot.pivot.slides.runManual(gp2.getLeftY());

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.pivot.slides.runToMM(0);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.pivot.slides.runToMM(240);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.pivot.slides.runToMM(480);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.pivot.slides.runToMM(710);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                pivotAngle += 5;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                pivotAngle -= 5;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.pivot.runToDeg(pivotAngle);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && bot.pivot.testing) {
                bot.pivot.setTesting(false);
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.pivot.setTesting(true);
            }

            if (Math.abs(gp1.getLeftY()) > 0.05) {
                bot.pivot.runManualIK(gp1.getLeftY());
            }

//            if (Math.abs(gp1.getRightY()) > 0.05) {
//                bot.pivot.manualRunTo(bot.pivot.getPosition() + gp1.getRightY() * 40);
//            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.pivot.lowChamber();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.pivot.highChamber();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.pivot.frontIntake();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.pivot.highBucket();
            }

            // DRIVE
            //drive();

            // TELEMETRY
            telemetry.addData("Pivot Active", bot.pivot.testing);
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot Teleop Target Angle", pivotAngle);
            telemetry.addData("Pivot IK Target Angle", bot.pivot.pivotIKTargetDegrees);
            //telemetry.addData("Pivot Motor Current", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Pivot PID", bot.pivot.power - bot.pivot.calculateFeedForward());
            //telemetry.addData("Pivot Manual Power Up", bot.pivot.manualPowerUp);
            //telemetry.addData("Pivot Target (ticks)", bot.pivot.getTarget());
            //telemetry.addData("Pivot Profiler", bot.pivot.getProfilerTarget());
            telemetry.addData("Pivot Power", bot.pivot.power);
            telemetry.addData("Slides Position (mm)", bot.pivot.slides.getmmPosition());
            telemetry.addData("Slides IK Position (mm)", bot.pivot.slides.getIKmmPosition());
            telemetry.addData("Slides Target (ticks)", bot.pivot.slides.getTarget());
            telemetry.addData("Slides Profiler", bot.pivot.slides.getProfilerTarget());
            telemetry.addData("Slides Power", bot.pivot.slides.power);
            telemetry.addData("Target X", bot.pivot.targetX);
            telemetry.addData("Target Z", bot.pivot.targetZ);
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