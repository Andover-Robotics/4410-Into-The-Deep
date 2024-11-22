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

    private int trackingCycler = 0;


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
        bot.state = Bot.BotState.FRONT_INTAKE;

        while (!isStarted()) {
            bot.pivot.periodic();
        }

        bot.frontIntake();

        while (opModeIsActive() && !isStopRequested()) {

            gp1.readButtons();
            gp2.readButtons();

            // left trigger for red, right trigger for blue alliance
            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 ) {
                sampleTracker.setRedAlliance();
            }

            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 || gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 ) {
                sampleTracker.setBlueAlliance();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.wallIntake();
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                bot.storage();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                sampleTracker.detectYellow = !sampleTracker.detectYellow;
            }


            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                if (sampleTracker.alliance == SampleTrackingPipeline.Alliance.RED) {
                    if (sampleTracker.detectYellow) {
                        if (sampleTracker.getAngleYellow() % 1 != 0) {
                            bot.pivot.arm.incrementRoll(sampleTracker.getAngleYellow());
                        }
                    } else {
                        if (sampleTracker.getAngleRed() % 1 != 0) {
                            bot.pivot.arm.incrementRoll(sampleTracker.getAngleRed());
                        }
                    }
                } else {
                    if (sampleTracker.detectYellow) {
                        if (sampleTracker.getAngleYellow() % 1 != 0) {
                            bot.pivot.arm.incrementRoll(sampleTracker.getAngleYellow());
                        }
                    } else {
                        if (sampleTracker.getAngleBlue() % 1 != 0) {
                            bot.pivot.arm.incrementRoll(sampleTracker.getAngleBlue());
                        }
                    }

                }
            }

            if (Math.abs(gp2.getLeftY()) > 0.05) {
                bot.pivot.runManualIK(gp2.getLeftY());
            }

//            if (gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
//                sampleTracker.detectYellow(false);
//            } else {
//                sampleTracker.detectYellow(true);
//            }
//
//            telemetry.addData("Current Pitch (Servo)", bot.pivot.arm.currentPitch);
//            telemetry.addData("Pitch Setpoint", bot.pivot.arm.pitchSetpoint);
//            telemetry.addData("Arm Passive IK", bot.pivot.armTesting);
            telemetry.addData("Red Angle", sampleTracker.getAngleRed());
            telemetry.addData("Blue Angle", sampleTracker.getAngleBlue());
            telemetry.addData("Yellow Angle", sampleTracker.getAngleYellow());
            telemetry.addData("Tracking Alliance", sampleTracker.getAlliance());
            telemetry.addData("Tracking Yellow", sampleTracker.detectYellow);
            telemetry.addData("Current Roll", bot.pivot.arm.rollSetpoint);
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