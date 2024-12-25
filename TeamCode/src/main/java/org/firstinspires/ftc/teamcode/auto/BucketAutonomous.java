package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "Bucket Autonomous", group = "Autonomous")
public class BucketAutonomous extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    boolean chamber = false, redAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        Pose2d initialPose = new Pose2d(9, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action preloadClip = drive.actionBuilder(drive.pose)
                .stopAndAdd(new SequentialAction(
                        bot.actionHighChamber()
                ))
                .strafeToLinearHeading(new Vector2d(8, 36), Math.toRadians(-90))

                .stopAndAdd(bot.actionFirstClipStorage())

                .afterDisp(7, new SequentialAction(
                        bot.actionSecondClipToIntake()
                ))
                .strafeToLinearHeading(new Vector2d(53,45), Math.toRadians(-90)) //first intake pos

                .stopAndAdd(new SleepAction(0.2))

                .build();

        Action preloadSample = drive.actionBuilder(drive.pose)
                .afterTime(0.01, bot.actionHighBucket())
                .strafeToLinearHeading(new Vector2d(59.5,56), Math.toRadians(-135))

                .waitSeconds(0.8)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.1),
                        bot.actionFrontIntake()
                ))

                .strafeToLinearHeading(new Vector2d(49.5,43.5), Math.toRadians(-90)) //first intake pos
                .stopAndAdd(new SleepAction(0.8))

                .build();

        Action bucketAuto = drive.actionBuilder(new Pose2d(53, 45, -90))
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.2)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(56,56), Math.toRadians(-135))

                .waitSeconds(1.15)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.1),
                        bot.actionFrontIntake()
                ))

                .strafeToLinearHeading(new Vector2d(60.5,42), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.5),
                        bot.actionPickDown(),
                        new SleepAction(0.2)))


                .afterTime(0.01, new SequentialAction(
                        bot.actionHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(56,56), Math.toRadians(-135))

                .waitSeconds(1.15)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.2),
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw()

                ))
                .strafeToLinearHeading(new Vector2d(55, 37.5), Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 20))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.2),
                        bot.actionPickDown(),
                        new SleepAction(0.3)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionHighBucket()
                ))
                .strafeToLinearHeading(new Vector2d(55,55), Math.toRadians(-135))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.8),
                        bot.actionBucketDrop(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.05),
                        bot.actionSubAutoIntake()
                ))
                .splineToLinearHeading(new Pose2d(new Vector2d(22, 2), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-12, 70))

                .build();

        Action finalBucket = drive.actionBuilder(new Pose2d(55, 55, Math.toRadians(-135)))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.5),
                        bot.actionBucketDrop(),
                        new SleepAction(0.1)

                ))
                .afterTime(0.01, bot.actionBucketToStorage())
                .splineToLinearHeading(new Pose2d(new Vector2d(20, 2), Math.toRadians(180)), Math.toRadians(180))
                .stopAndAdd(bot.actionSubPark())
                .waitSeconds(1)
                .build();

        bot.openPipeline(true, false, true);

        while(!isStarted()) {
            bot.pivot.periodic();

            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                chamber = !chamber;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                redAlliance = !redAlliance;
                bot.pipeline.setBlue(!redAlliance);
                bot.pipeline.setRed(redAlliance);
            }

            telemetry.addData("CLIPPING (A)", chamber);
            if (redAlliance) {
                telemetry.addLine("RED ALLIANCE (B)");
            } else {
                telemetry.addLine("BLUE ALLIANCE (B)");
            }
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                chamber? preloadClip : preloadSample,
                                bucketAuto,
                                bot.actionDetectWait()
                        )
                )
        );

        bot.updateSampleDrive();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionDetect(),
                                new SleepAction(0.6),
                                drive.actionBuilder(new Pose2d(22, 2, Math.toRadians(180)))
                                        .strafeToConstantHeading(new Vector2d(22, 2 + Bot.sampleYPos))
                                        .stopAndAdd(new SequentialAction(
                                                bot.actionSubAutoPickDown(),
                                                new SleepAction(0.3),
                                                bot.actionSubAutoPickUp(),
                                                new SleepAction(0.1)
                                        ))
                                        .afterTime(0.01, new SequentialAction(
                                                bot.actionFrontIntakeToStorage(),
                                                bot.actionHighBucket()))
                                        .splineToLinearHeading(new Pose2d(new Vector2d(52, 55), Math.toRadians(-135)), Math.toRadians(-179))
                                        .build(),
                                finalBucket
                        ),
                        telemetryPacket -> {
                            telemetry.addData("sample y val", Bot.sampleYPos);
                            telemetry.update();
                            return true;
                        }
                )
        );
    }
}
