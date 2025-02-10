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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "Testing Bucket Autonomous", group = "Autonomous")
public class TestingBucketAutonomous extends LinearOpMode {
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

        Pose2d initialPose = new Pose2d(9, 63, Math.toRadians(-90)); //TODO NEED TO CHANGE TO ONE TILE OVER

        Pose2d firstSample = new Pose2d(47.3, 47, Math.toRadians(-90));
        Pose2d secondSample = new Pose2d(61, 44.25, Math.toRadians(-88));
        Pose2d thirdSample = new Pose2d(57, 39.5, Math.toRadians(-45));

        Pose2d firstBucket = new Pose2d(54.5, 59, Math.toRadians(-135));
        Pose2d secondBucket = new Pose2d(54, 58.5, Math.toRadians(-135));
        Pose2d thirdBucket = new Pose2d(52, 56.5, Math.toRadians(-135));

        Pose2d firstCVSample = new Pose2d(firstSample.component1().x, firstSample.component1().y - 4, Math.toRadians(-90));
        Pose2d secondCVSample = new Pose2d(secondSample.component1().x, secondSample.component1().y - 4, Math.toRadians(-90));

        Pose2d firstSub = new Pose2d(24, 3, Math.toRadians(180));
        Pose2d secondSub = new Pose2d(24, 3, Math.toRadians(180));
        Pose2d firstSubDrop = new Pose2d(55, 63.5, Math.toRadians(-135));
        Pose2d secondSubDrop = new Pose2d(55, 63.5, Math.toRadians(-135));

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

                .strafeToLinearHeading(firstSample.component1(), firstSample.component2()) //first intake pos

                .build();

        Action firstSamplePickup = drive.actionBuilder(firstSample)
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.05),
                        bot.actionPickUp(),
                        new SleepAction(0.1)
                ))
                .build();

        Action firstSampleCVPickup = drive.actionBuilder(firstSample)
                .afterTime(0.01, bot.actionSubAutoIntake())
                .strafeToLinearHeading(firstCVSample.component1(), firstCVSample.component2())
                .build();

        Action firstSampleSetDrop = drive.actionBuilder(firstSample)
                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))

                .strafeToLinearHeading(firstBucket.component1(), firstBucket.component2())

                .waitSeconds(0.35)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))
                .strafeToLinearHeading(secondSample.component1(), secondSample.component2())
                .build();

        Action secondSamplePickup = drive.actionBuilder(secondSample)
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.05),
                        bot.actionPickUp(),
                        new SleepAction(0.1)
                ))
                .build();

        Action secondSampleCVPickup = drive.actionBuilder(secondSample)
                .afterTime(0.01, bot.actionSubAutoIntake())
                .strafeToLinearHeading(secondCVSample.component1(), secondCVSample.component2())
                .build();

        Action secondSampleSetDrop = drive.actionBuilder(secondSample)
                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))

                .strafeToLinearHeading(secondBucket.component1(), secondBucket.component2())

                .waitSeconds(0.35)
                .stopAndAdd(bot.actionBucketDrop())
                .build();

        Action thirdSampleToSub = drive.actionBuilder(secondBucket)
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.1),
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw()
                ))

                .strafeToLinearHeading(thirdSample.component1(), thirdSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 45))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.20),
                        bot.actionPickDown(),
                        new SleepAction(0.25)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))
                .strafeToLinearHeading(thirdBucket.component1(), thirdBucket.component2())
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.2),
                        bot.actionBucketDrop()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        bot.actionSubAutoIntake()
                ))
                .splineToLinearHeading(firstSub, Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-12, 350))
                .build();

        Action backToSub = drive.actionBuilder(firstSubDrop)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.17),
                        bot.actionBucketDrop(),
                        bot.actionResetPipeline()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.05),
                        bot.actionSubAutoIntake()
                ))
                .splineToLinearHeading(secondSub, Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-12, 350))
                .build();

        Action finalBucket = drive.actionBuilder(secondSubDrop)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.17),
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
                                bot.actionResetPipeline(),
                                preloadSample,
                                firstSamplePickup
                        )
                )
        );

        if (bot.isEmpty()) {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            bot.actionResetPipeline(),
                            firstSampleCVPickup,
                            bot.actionDetectWait()
                    )

            ));

            bot.updateSampleDrive();

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    drive.actionBuilder(firstCVSample)
                                            .strafeToConstantHeading(new Vector2d(firstCVSample.component1().x - Bot.sampleYPos, firstCVSample.component1().y))
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .strafeToLinearHeading(firstBucket.component1(), firstBucket.component2())
                                            .waitSeconds(0.4)
                                            .stopAndAdd(bot.actionBucketDrop())

                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionBucketToFrontIntake()
                                            ))
                                            .strafeToLinearHeading(secondSample.component1(), secondSample.component2())
                                            .build()
                            )
                    )
            );
        } else {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            firstSampleSetDrop
                    )

            ));
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                secondSamplePickup
                        )
                )
        );

        if (bot.isEmpty()) {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            bot.actionResetPipeline(),
                            secondSampleCVPickup,
                            bot.actionDetectWait()
                    )

            ));

            bot.updateSampleDrive();

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    drive.actionBuilder(secondCVSample)
                                            .strafeToConstantHeading(new Vector2d(secondCVSample.component1().x - Bot.sampleYPos, secondCVSample.component1().y))
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .strafeToLinearHeading(secondBucket.component1(), secondBucket.component2())
                                            .waitSeconds(0.4)
                                            .stopAndAdd(bot.actionBucketDrop())
                                            .build()
                            )
                    )
            );
        } else {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            secondSampleSetDrop
                    )

            ));
        }

        Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                bot.actionPeriodic(),
                new SequentialAction(
                        bot.actionResetPipeline(),
                        thirdSampleToSub,
                        bot.actionDetectWait()
                )

        ));

        // TODO position failsafes (ramming if position is blocked)
//        if (bot.vectorDiff(drive.pose.position, firstSub.component1()) > 2.5) {
//            Actions.runBlocking(
//                    new ActionHelper.RaceParallelCommand(
//                            bot.actionPeriodic(),
//                            new SequentialAction(
//                                    bot.actionFrontIntakeToStorage(),
//                                    drive.actionBuilder(drive.pose)
//
//                            )
//                    )
//            );
//        }

        bot.updateSampleDrive();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionDetect(),
                                drive.actionBuilder(firstSub)
                                        .strafeToConstantHeading(new Vector2d(firstSub.component1().x, firstSub.component1().y + Bot.sampleYPos))
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
                                        .splineToLinearHeading(firstSubDrop, Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

                                        .build(),
                                backToSub,
                                bot.actionResetPipeline(),
                                bot.actionDetectWait()
                        ),
                        telemetryPacket -> {
                            telemetry.addData("sample y val", Bot.sampleYPos);
                            telemetry.update();
                            return true;
                        }
                )
        );

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionResetPipeline(),
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
                                drive.actionBuilder(secondSub)
                                        .strafeToConstantHeading(new Vector2d(secondSub.component1().x, secondSub.component1().y + Bot.sampleYPos))
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
                                        .splineToLinearHeading(firstSubDrop, Math.toRadians(-175), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 300))

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
