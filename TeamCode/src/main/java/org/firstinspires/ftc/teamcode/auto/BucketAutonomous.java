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

        Pose2d initialPose = new Pose2d(12, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // new Pose2d(10, 58, Math.toRadians(-90))
        Action bucketAuto = drive.actionBuilder(drive.pose)

                .afterTime(0.01, bot.actionHighChamber())
                .strafeToLinearHeading(new Vector2d(10.5, 37), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

//                .afterTime(0.58, bot.actionFrontIntake())
                .strafeToLinearHeading(new Vector2d(49,44), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        new SleepAction(0.4),
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.1, new SequentialAction(
                        bot.actionHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(55,59), Math.toRadians(-135))

                .waitSeconds(0.65)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
                ))

                .strafeToLinearHeading(new Vector2d(57.9,45), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        new SleepAction(0.5),
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.1, new SequentialAction(
                        bot.actionHighBucket()
                ))
                .strafeToLinearHeading(new Vector2d(55.75,59.75), Math.toRadians(-135))

                .waitSeconds(0.65)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
                ))
                .strafeToLinearHeading(new Vector2d(54.75, 39.5), Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 20))

                .stopAndAdd(new SequentialAction(
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw(),
                        new SleepAction(0.4),
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp()))

                .afterTime(0.01, new SequentialAction(
                        bot.actionFrontIntakeToStorage()
                ))
                .strafeToLinearHeading(new Vector2d(55,59), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 20))
                .stopAndAdd(new SequentialAction(
                        bot.actionHighBucket(),
                        new SleepAction(0.65),
                        bot.actionBucketDrop(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
                ))
                .strafeToLinearHeading(new Vector2d(46,14), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(20.5,14), Math.toRadians(180))
                .stopAndAdd(bot.actionSubPark())
                .waitSeconds(5)

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
