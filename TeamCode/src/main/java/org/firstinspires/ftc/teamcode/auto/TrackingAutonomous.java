package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
                        bot.actionFrontIntake(),
                        new SleepAction(0.5),
                        new InstantAction(() -> bot.scan()),
                        new SleepAction(0.2),
                        new InstantAction(() -> bot.alignClaw()),
                        new InstantAction(() -> bot.updateSampleDrive()),
                        new InstantAction(() -> bot.updateSampleSlides())
                ))
                .build();

        TrajectoryActionBuilder autoIntake = drive.actionBuilder(new Pose2d(20.5, startingY, Math.toRadians(180)))

                .lineToYConstantHeading(bot.sampleYPos)
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.3),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.5)
                ));


        while(!isStarted()) {
            bot.pivot.periodic();

            gp1.readButtons();

            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(autoDetect,
                                autoIntake.build())
                )
        );
    }
}
