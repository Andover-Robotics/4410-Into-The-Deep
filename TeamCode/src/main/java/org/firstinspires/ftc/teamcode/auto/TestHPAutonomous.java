package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "HP Autonomous", group = "Autonomous")
public class TestHPAutonomous extends LinearOpMode {
    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();


        // red big init pose NOTE: check comment above each trajectory to find the respective init pose
        Pose2d initialPose = new Pose2d(-10, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action hpAuto = drive.actionBuilder(drive.pose)
                .afterTime(0.1, bot.actionHighChamber())  // First chamber clip
                .strafeToLinearHeading(new Vector2d(-8, 39.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                .afterTime(0.5, bot.actionFrontIntake()) // Pick up first sample

                .strafeToLinearHeading(new Vector2d(-51,45), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.1),
                        bot.actionWallIntakeClosed(),
                        new SleepAction(2),
                        bot.actionOpenGripper(),
                        new SleepAction(0.6),
                        bot.actionWallIntakeToStorage()
                ))

                .afterTime(0.1, bot.actionFrontIntake()) // Pick up second sample
                .strafeToLinearHeading(new Vector2d(-59,44), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.6),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.1),
                        bot.actionWallIntakeClosed(),
                        new SleepAction(2),
                        bot.actionOpenGripper()
                ))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-59,50),Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                //high chamber
                .strafeToLinearHeading(new Vector2d(-4,39.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                // Pick up third sample
                .strafeToLinearHeading(new Vector2d(-40,39), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(-52,26), Math.toRadians(-180))
                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        bot.actionRotateClaw(),
                        new SleepAction(0.4),
                        bot.actionPickDown(),
                        new SleepAction(0.4),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage()
                ))

                // Drop third sample in HP zone
                .afterTime(0.3, bot.actionWallIntakeClosed())
                .strafeToLinearHeading(new Vector2d(-59,44),Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.5),
                        bot.actionOpenGripper()
                ))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-59,50),Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                .strafeToLinearHeading(new Vector2d(-48,12), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(-19.5,12), Math.toRadians(-180))
                .build();

        while(!isStarted()) {
            bot.pivot.periodic();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.periodic(),
                        hpAuto
                )
        );
    }
}
