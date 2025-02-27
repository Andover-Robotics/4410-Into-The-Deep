package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
@Autonomous(name = "Old Bucket Autonomous", group = "Old Autonomous")
public class OldBucketAutonomous extends LinearOpMode {
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

        Action preloadSample = drive.actionBuilder(drive.pose)
                .afterTime(0.01, bot.actionHighBucket())
                .strafeToLinearHeading(new Vector2d(59.5,55), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionBucketDrop()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))

                .strafeToLinearHeading(new Vector2d(47.3,47), Math.toRadians(-90)) //first intake pos

                .build();

        Action bucketAuto = drive.actionBuilder(new Pose2d(47.3, 47, -90))
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(54.5,59), Math.toRadians(-135))

                .waitSeconds(0.35)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))

                .strafeToLinearHeading(new Vector2d(61,44.25), Math.toRadians(-88))

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.1)
                ))


                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))

                .strafeToLinearHeading(new Vector2d(54,58.5), Math.toRadians(-135))

                .waitSeconds(0.35)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.1),
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw()
                ))

                .strafeToLinearHeading(new Vector2d(57, 39.5), Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 85))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.20),
                        bot.actionPickDown(),
                        new SleepAction(0.25)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))
                .strafeToLinearHeading(new Vector2d(52,56.5), Math.toRadians(-135))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.2),
                        bot.actionBucketDrop()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        bot.actionSubAutoIntake()
                ))
                .splineToLinearHeading(new Pose2d(new Vector2d(24, 3), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-12, 350))
                .build();

        Action backToSub = drive.actionBuilder(new Pose2d(55, 63.5, Math.toRadians(-135)))
//
//                .afterTime(0.01, new SequentialAction(
//                        bot.actionIntakeToHighBucket()
//                ))
//                .strafeToLinearHeading(new Vector2d(44, 8), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//                .splineToLinearHeading(new Pose2d(new Vector2d(53.5, 70), Math.toRadians(-135)), Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.1),
                        bot.actionBucketDrop(),
                        bot.actionResetPipeline()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.05),
                        bot.actionSubAutoIntake()
                ))
                .splineToLinearHeading(new Pose2d(new Vector2d(24, -4), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-12, 350))
                .build();

        Action finalBucket = drive.actionBuilder(new Pose2d(55.25, 65, Math.toRadians(-135)))
//                .afterTime(0.01, new SequentialAction(
//                        bot.actionIntakeToHighBucket()
//                ))
//                .strafeToLinearHeading(new Vector2d(44, 8), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//                .splineToLinearHeading(new Pose2d(new Vector2d(54, 64), Math.toRadians(-135)), Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

                .stopAndAdd(new SequentialAction(
                        bot.actionBucketDrop()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
//                        new SleepAction(0.8),
//                        bot.actionSubPark()
                ))
//                .splineToLinearHeading(new Pose2d(new Vector2d(20, 2), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 350))
                .waitSeconds(5)
                .build();

        bot.openPipeline(true, false, true);

        while(!isStarted()) {
            bot.autoPeriodic();

            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                redAlliance = !redAlliance;
                bot.pipeline.setBlue(!redAlliance);
                bot.pipeline.setRed(redAlliance);
            }

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
                                preloadSample,
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
                                drive.actionBuilder(new Pose2d(24, 3, Math.toRadians(180)))
                                        .strafeToConstantHeading(new Vector2d(24, 3 + Bot.sampleYPos))
                                        .stopAndAdd(new SequentialAction(
                                                bot.actionSubAutoPickDown(),
                                                new SleepAction(0.25),
                                                bot.actionSubAutoPickUp()
                                        ))
                                        .afterTime(0.01, new SequentialAction(
                                                bot.actionFrontIntakeToStorage(),
                                                new SleepAction(0.25),
                                                bot.actionHighBucket()
                                        ))
                                        .strafeToLinearHeading(new Vector2d(44, 8), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                                        .splineToLinearHeading(new Pose2d(new Vector2d(55, 63.5), Math.toRadians(-135)), Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

                                        .build(),
                                backToSub,
                                bot.actionDetectWait()
                        ),
                        telemetryPacket -> {
                            telemetry.addData("sample y val", Bot.sampleYPos);
                            telemetry.update();
                            return true;
                        }
                )
        );

        bot.updateSampleDrive();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionDetect(),
                                drive.actionBuilder(new Pose2d(24, -4, Math.toRadians(180)))
                                        .strafeToConstantHeading(new Vector2d(24, -4 + Bot.sampleYPos))
                                        .stopAndAdd(new SequentialAction(
                                                bot.actionSubAutoPickDown(),
                                                new SleepAction(0.25),
                                                bot.actionSubAutoPickUp()
                                        ))
                                        .afterTime(0.01, new SequentialAction(
                                                bot.actionFrontIntakeToStorage(),
                                                new SleepAction(0.25),
                                                bot.actionHighBucket()
                                        ))
                                        .strafeToLinearHeading(new Vector2d(44, 8), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                                        .splineToLinearHeading(new Pose2d(new Vector2d(55.25, 65), Math.toRadians(-135)), Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

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
