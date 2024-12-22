package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "tracking tester", group = "Autonomous")
public class TrackingAutonomous extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    double startingY = 14;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        Pose2d initialPose = new Pose2d(20.5, startingY, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action autoDetect = drive.actionBuilder(new Pose2d(20.5, startingY, Math.toRadians(180)))
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> bot.updateSampleSlides()),
                        new InstantAction(() -> bot.alignClaw()),
                        new InstantAction(() -> bot.updateSampleDrive()),
                        new SleepAction(0.6)
                ))
                .build();

        TrajectoryActionBuilder autoIntake = drive.actionBuilder(new Pose2d(20.5, startingY, Math.toRadians(180)))

                //.strafeToConstantHeading(new Vector2d(20.5, Bot.sampleYPos + startingY))
                .stopAndAdd(new SequentialAction(
                        bot.actionSubAutoPickDown(),
                        new SleepAction(0.8),
                        bot.actionSubAutoPickUp(),
                        new SleepAction(0.3),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.5)
                ));


        bot.openPipeline(true, false, false);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    bot.actionSubAutoIntake()
                )
        );

        while(!isStarted()) {
            bot.pivot.periodic();
            bot.scan();

            gp1.readButtons();

            telemetry.addData("crrent roll", bot.pivot.arm.rollSetpoint);
            telemetry.addData("current alge", bot.pipeline.getAngle());
            telemetry.addData("current slides inch offset", bot.pipeline.getY());
            telemetry.addData("current drive inch offset", bot.pipeline.getX());
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
//                                bot.actionSubAutoIntake(),
//                                new SleepAction(0.6),
                                new InstantAction(() -> bot.scan()),
                                new SleepAction(0.1),
                                new InstantAction(() -> bot.scan()),
                                new SleepAction(0.05),
                                autoDetect,
                                autoIntake.build()
                                )
                )
        );
    }
}