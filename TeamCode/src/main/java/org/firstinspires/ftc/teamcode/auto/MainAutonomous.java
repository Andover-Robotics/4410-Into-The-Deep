package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {
    private Bot bot;
    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-63, -12, 0);

        Bot.instance = null;
        bot = Bot.getInstance(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action sample1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-44, -51))
                .build();

        Action sample2 = drive.actionBuilder(new Pose2d(-55, -52, Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-44, -60), 0)
                .build();

        Action sample3 = drive.actionBuilder(new Pose2d(-55, -52, Math.PI / 4))
                .strafeToLinearHeading(new Vector2d(-25, -54), -Math.PI / 2)
                .build();

        Action bucket1 = drive.actionBuilder(new Pose2d(-44, -51, 0))
                .strafeToLinearHeading(new Vector2d(-55, -52), Math.PI / 4)
                .build();

        Action bucket2 = drive.actionBuilder(new Pose2d(-44, -60, 0))
                .strafeToLinearHeading(new Vector2d(-55, -52), Math.PI / 4)
                .build();

        Action bucket3 = drive.actionBuilder(new Pose2d(-25, -54, -Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(-55, -52), Math.PI / 4)
                .build();

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        while(!isStarted()) {
            bot.pivot.periodic();
        }

        Actions.runBlocking(
                new ParallelAction(
                        bot.periodic(),
                        new SequentialAction(
                                sample1,
                                bot.actionFrontIntake(),
                                new SleepAction(0.5),
                                bot.actionPickDown(),
                                new SleepAction(0.3),
                                bot.actionPickUp(),
                                new SleepAction(0.4),
                                bot.actionFrontIntakeToStorage(),
                                new ParallelAction(bucket1, new SequentialAction (new SleepAction(0.2), bot.actionHighBucket())),
                                new SleepAction(0.4),
                                bot.actionBucketDrop(),
                                new SleepAction(1),
                                sample2,
                                bot.actionFrontIntake(),
                                new SleepAction(0.5),
                                bot.actionPickDown(),
                                new SleepAction(0.3),
                                bot.actionPickUp(),
                                new SleepAction(0.4),
                                bot.actionFrontIntakeToStorage(),
                                new ParallelAction(bucket2, new SequentialAction (new SleepAction(0.2), bot.actionHighBucket())),
                                new SleepAction(0.4),
                                bot.actionBucketDrop(),
                                new SleepAction(1),
                                sample3,
                                bot.actionFrontIntake(),
                                bot.actionRotateClaw(),
                                new SleepAction(0.3),
                                bot.actionPickDown(),
                                new SleepAction(0.15),
                                bot.actionPickUp(),
                                new SleepAction(0.2),
                                bot.actionFrontIntakeToStorage(),
                                new ParallelAction(bucket3, new SequentialAction (new SleepAction(0.2), bot.actionHighBucket())),
                                new SleepAction(0.4),
                                bot.actionBucketDrop(),
                                new SleepAction(1)
                        )
                )
        );
    }
}
