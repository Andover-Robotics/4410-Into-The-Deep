package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "Bucket Autonomous", group = "Autonomous")
public class BucketAutonomous extends LinearOpMode {
    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);


        bot.state = Bot.BotState.STORAGE;
        bot.storage();


        // red big init pose NOTE: check comment above each trajectory to find the respective init pose
        Pose2d initialPose = new Pose2d(10, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // new Pose2d(10, 58, Math.toRadians(-90))
        Action bucketAuto = drive.actionBuilder(drive.pose)

                .stopAndAdd(bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(8.5, 39.1), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.2),
                        bot.actionClipStorage()
                ))
                .afterTime(0.5, bot.actionFrontIntake())
                .strafeToLinearHeading(new Vector2d(48,46), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage()))

                .afterTime(0.2, bot.actionHighBucket())
                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135))

                .waitSeconds(1)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.1, bot.actionFrontIntake())
                .strafeToLinearHeading(new Vector2d(56,44.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage()))

                .afterTime(0.2, bot.actionHighBucket())
                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135))

                .waitSeconds(1)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.8, new SequentialAction(
                        new SleepAction(0.4),
                        bot.actionFrontIntake(),
                        bot.actionRotateClaw()
                ))
                .strafeToLinearHeading(new Vector2d(49,28), Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage()
                ))

                .afterTime(0.2, bot.actionHighBucket())
                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135))

                .waitSeconds(0.3)
                .stopAndAdd(bot.actionBucketDrop())

                .strafeToLinearHeading(new Vector2d(48,12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(18.5,12), Math.toRadians(180))
                .stopAndAdd(new InstantAction(() -> bot.pivot.arm.vertical()))

                .build();

        while(!isStarted()) {
            bot.pivot.periodic();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.periodic(),
                        bucketAuto
                )
        );
    }
}
