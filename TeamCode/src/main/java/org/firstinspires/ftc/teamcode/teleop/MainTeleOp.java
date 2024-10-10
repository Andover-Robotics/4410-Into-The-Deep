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

        double pivotAngle = 90;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if (gp1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                pivotAngle += 2.5;
            }

            if (gp1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                pivotAngle -= 2.5;
            }

            if (gp1.wasJustReleased(GamepadKeys.Button.A) && bot.pivot.testing) {
                bot.pivot.setTesting(true);
            } else if (gp1.wasJustReleased(GamepadKeys.Button.A)) {
                bot.pivot.setTesting(false);
            }

            // TELEMETRY
            telemetry.addData("Pivot Ticks", bot.pivot.getPosition());
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot Radians", bot.pivot.getPivotAngleRadians());
            telemetry.addData("Pivot Active", bot.pivot.testing);
            telemetry.addData("Pivot Target Angle", pivotAngle);
            telemetry.addData("Pivot Motor Power", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
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