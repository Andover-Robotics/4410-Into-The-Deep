package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

                .afterTime(0.1, bot.actionHighChamber())
                .strafeToLinearHeading(new Vector2d(8.5, 39.1), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.15),
                        bot.actionClipStorage()
                ))

                .afterTime(0.5, bot.actionFrontIntake())
                .strafeToLinearHeading(new Vector2d(47,44.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.2, new SequentialAction(
                        bot.actionHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135))

                .waitSeconds(0.9)
                .stopAndAdd(new SequentialAction(
                        bot.actionBucketDrop(),
                        new SleepAction(0.2),
                        bot.actionBucketToStorage(),
                        new SleepAction(0.2)))

                .afterTime(0.2, bot.actionFrontIntake())
                .strafeToLinearHeading(new Vector2d(56,44.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.2, new SequentialAction(
                        bot.actionHighBucket()
                ))
                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135))

                .waitSeconds(0.9)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
                ))
                .strafeToLinearHeading(new Vector2d(49,28), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 20))

                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        bot.actionRotateClaw(),
                        new SleepAction(0.6),
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.01, new SequentialAction(
                        bot.actionFrontIntakeToStorage()
                ))
                .strafeToLinearHeading(new Vector2d(53.5,57.5), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 20))
                .stopAndAdd(new SequentialAction(
                        bot.actionHighBucket(),
                        new SleepAction(0.8),
                        bot.actionBucketDrop(),
                        new SleepAction(0.1),
                        bot.actionBucketToStorage()
                ))

                .strafeToLinearHeading(new Vector2d(48,12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(18.5,12), Math.toRadians(180))
                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> bot.pivot.arm.vertical()),
                        new SleepAction(5)))

                .build();

        while(!isStarted()) {
            bot.pivot.periodic();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        bucketAuto
                )
        );
    }
}
