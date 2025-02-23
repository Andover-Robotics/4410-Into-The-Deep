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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "Bucket Auto (6 Sample)", group = "Autonomous")
public class BucketAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    boolean chamber = false, redAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        bot.state = Bot.BotState.STORAGE;
        bot.initialize();

        int cv = 0;

        Pose2d initialPose = new Pose2d(33, 63, Math.toRadians(-90)); //ONE TILE OVER
        Vector2d preloadDrop = new Vector2d(60, 53.5);
        Pose2d firstSample = new Pose2d(48, 47, Math.toRadians(-89));
        Pose2d secondSample = new Pose2d(60.5, 46.2, Math.toRadians(-89.5));
        Pose2d thirdSample = new Pose2d(56.5, 38, Math.toRadians(-45));

        Pose2d firstBucket = new Pose2d(56.5, 57, Math.toRadians(-135));
        Pose2d secondBucket = new Pose2d(56, 56, Math.toRadians(-135));
        Pose2d thirdBucket = new Pose2d(54, 54.5, Math.toRadians(-135));

        Pose2d firstCVSample = new Pose2d(firstSample.component1().x, firstSample.component1().y - 2, Math.toRadians(-90));
        Pose2d secondCVSample = new Pose2d(secondSample.component1().x, secondSample.component1().y - 2, Math.toRadians(-90));
        Pose2d thirdCVSample = new Pose2d(52.5, 24, Math.toRadians(0));

        Pose2d firstSub = new Pose2d(21, 3.5, Math.toRadians(180));
        Pose2d secondSub = new Pose2d(21, 6, Math.toRadians(180));
        Pose2d firstSubDrop = new Pose2d(52.5, 55, Math.toRadians(-135));
        Pose2d secondSubDrop = new Pose2d(52.5, 55, Math.toRadians(-135));

        Pose2d subDropRam = new Pose2d(48, 7, Math.toRadians(180));
        Pose2d unRam = new Pose2d(23, 7, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        Action preloadSample = drive.actionBuilder(drive.pose)
                .afterTime(0.01, bot.actionHighBucket())
                .strafeToSplineHeading(preloadDrop, Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 110))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.12),
                        bot.actionBucketDrop()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))

                .strafeToLinearHeading(firstSample.component1(), firstSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-35, 50)) //first intake pos

                .build();

        Action firstSamplePickup = drive.actionBuilder(firstSample)
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.05),
                        bot.actionPickUp(),
                        new SleepAction(0.20)
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

                .waitSeconds(0.30)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))
                .strafeToLinearHeading(secondSample.component1(), secondSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-35, 40))
                .build();

        Action secondSamplePickup = drive.actionBuilder(secondSample)
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.15),
                        bot.actionPickUp(),
                        new SleepAction(0.20)
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

                .waitSeconds(0.30)
                .stopAndAdd(bot.actionBucketDrop())

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.1),
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw(),
                        bot.actionOpenGripper()
                ))

                .strafeToLinearHeading(thirdSample.component1(), thirdSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 40))
                .build();


        Action thirdSamplePickup = drive.actionBuilder(thirdSample)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.1),
                        bot.actionPickDown(),
                        new SleepAction(0.1),
                        bot.actionPickUp(),
                        new SleepAction(0.25)
                ))
                .build();

        Action thirdSampleCVPickup = drive.actionBuilder(thirdSample)
                .afterTime(0.01, bot.actionSubAutoIntake())
                .strafeToLinearHeading(thirdCVSample.component1(), thirdCVSample.component2())
                .build();

        Action thirdSampleSetDropToSub = drive.actionBuilder(thirdSample)
                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket()
                ))
                .strafeToLinearHeading(thirdBucket.component1(), thirdBucket.component2())
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.15),
                        bot.actionBucketDrop()
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        bot.actionSubAutoIntake()
                ))
                .splineToSplineHeading(new Pose2d(new Vector2d(firstSub.component1().x + 10, firstSub.component1().y), Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(firstSub, Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 350))
                .build();

        Action backToSub = drive.actionBuilder(firstSubDrop)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.12),
                        bot.actionBucketDrop(),
                        bot.actionResetPipeline()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        new SleepAction(0.05),
                        bot.actionSubAutoIntake()
                ))
                .splineToSplineHeading(new Pose2d(new Vector2d(secondSub.component1().x + 10, secondSub.component1().y), Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(secondSub, Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 350))
                .build();

        Action backToSubPark = drive.actionBuilder(firstSubDrop)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.12),
                        bot.actionBucketDrop(),
                        bot.actionResetPipeline()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
//                        new SleepAction(0.05),
//                        bot.actionSubPark()
                ))
                .splineToSplineHeading(new Pose2d(new Vector2d(secondSub.component1().x + 10, secondSub.component1().y), Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(secondSub.component1().x-4, secondSub.component1().y), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 350))
                .stopAndAdd(bot.actionSubPark())
                .build();

        Action finalBucket = drive.actionBuilder(secondSubDrop)
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.12),
                        bot.actionBucketDrop()
                ))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage()
//                        new SleepAction(0.8),
//                        bot.actionSubPark()
                ))
//                .splineToLinearHeading(new Pose2d(new Vector2d(20, 2), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 350))
//                .waitSeconds(5)
                .build();

        bot.openPipeline(false, false, true);

        while (!isStarted()) {
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
            cv++;
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            bot.actionResetPipeline(),
                            firstSampleCVPickup,
                            new SleepAction(0.7),
                            bot.actionDetectWait()
                    )

            ));

            bot.updateSampleDrive();
            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    controller.cvp2p(),
                                    new InstantAction(() -> bot.savePosition(drive.pose)),
                                    drive.actionBuilderPrecise(bot.storedPosition)
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
                                            .build(),
                                    secondSamplePickup
                            )
                    )
            );
        } else {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            firstSampleSetDrop,
                            secondSamplePickup
                    )

            ));
        }


        if (bot.isEmpty()) {
            cv++;
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            bot.actionResetPipeline(),
                            secondSampleCVPickup,
                            new SleepAction(0.7),
                            bot.actionDetectWait()
                    )

            ));

            bot.updateSampleDrive();
            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    controller.cvp2p(),
                                    new InstantAction(() -> bot.savePosition(drive.pose)),
                                    drive.actionBuilderPrecise(bot.storedPosition)
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

                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionBucketToStorage(),
                                                    new SleepAction(0.1),
                                                    bot.actionDiagFrontIntake(),
                                                    bot.actionRotateClaw()
                                            ))

                                            .strafeToLinearHeading(thirdSample.component1(), thirdSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 40))

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
                        thirdSamplePickup
                )

        ));

        if (bot.isEmpty()) {
            cv++;
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            bot.actionResetPipeline(),
                            thirdSampleCVPickup,
                            new SleepAction(0.7),
                            bot.actionDetectWait()
                    )

            ));

            bot.updateSampleDrive();
            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    controller.cvp2p(),
                                    new InstantAction(() -> bot.savePosition(drive.pose)),
                                    drive.actionBuilderPrecise(bot.storedPosition)
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .strafeToLinearHeading(thirdBucket.component1(), thirdBucket.component2())
                                            .stopAndAdd(new SequentialAction(
                                                    new SleepAction(0.15),
                                                    bot.actionBucketDrop()
                                            ))

                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionBucketToStorage(),
                                                    bot.actionSubAutoIntake()
                                            ))
                                            .splineToSplineHeading(new Pose2d(new Vector2d(firstSub.component1().x + 10, firstSub.component1().y), Math.toRadians(180)), Math.toRadians(180))
                                            .splineToLinearHeading(firstSub, Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 350))
                                            .build()
                            )
                    )
            );
        } else {
            Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    new SequentialAction(
                            thirdSampleSetDropToSub
                    )

            ));
        }

        Actions.runBlocking(new ActionHelpersJava.RaceParallelCommand(
                bot.actionPeriodic(),
                new SequentialAction(
                        bot.actionResetPipeline(),
                        new SleepAction(0.3),
                        bot.actionDetectWait()
                )

        ));

        bot.updateSampleDrive();
        drive.updatePoseEstimate();
        bot.savePosition(drive.pose);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionDetect(),
                                controller.cvp2p(),
                                new InstantAction(() -> bot.savePosition(drive.pose)),
                                drive.actionBuilderPrecise(bot.storedPosition)
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
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(44, 8, Math.toRadians(180)), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                                        .splineToSplineHeading(firstSubDrop, Math.toRadians(-150), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 300))

                                        .build()
                        ),
                        telemetryPacket -> {
                            telemetry.addData("sample y val", Bot.sampleYPos);
                            telemetry.update();
                            return true;
                        }
                )
        );

        //        // TODO position failsafes (ramming if position is blocked)
        drive.updatePoseEstimate();
        if (Bot.vectorDiff(drive.pose.position, firstSubDrop.component1()) > 18) {
            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionBucketToStorage()
                            )
                    ));
            while (Bot.vectorDiff(drive.pose.position, subDropRam.component1()) > 4) {
                drive.updatePoseEstimate();
                bot.savePosition(drive.pose);
                Actions.runBlocking(
                        new ActionHelper.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
//                                            controller.ramp2p(unRam),
//                                            controller.ramp2p(subDropRam)
                                        drive.actionBuilder(bot.storedPosition)
                                                .strafeToLinearHeading(firstSub.position, Math.toRadians(180))
                                                .splineTo(subDropRam.position, Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-300, 300))
                                                .build()
                                )
                        )
                );
                cv++;
            }

            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    drive.actionBuilderPrecise(bot.storedPosition)
                                            //
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionHighBucket()
                                            ))
                                            .setReversed(true)
                                            .splineToSplineHeading(firstSubDrop, Math.toRadians(-150), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 300))
                                            .build()
                            ),
                            telemetryPacket -> {
                                telemetry.addData("sample y val", Bot.sampleYPos);
                                telemetry.update();
                                return true;
                            }
                    )
            );
        }


        if (cv == 0) {
            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    backToSub,
                                    new SleepAction(0.4),
                                    bot.actionResetPipeline(),
                                    bot.actionDetectWait()
                            )
                    )
            );

            bot.updateSampleDrive();
            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    bot.actionDetect(),
                                    controller.cvp2p(),
                                    new InstantAction(() -> bot.savePosition(drive.pose)),
                                    drive.actionBuilderPrecise(bot.storedPosition)
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
                                            .setReversed(true)
                                            .splineToLinearHeading(new Pose2d(44, 8, Math.toRadians(180)), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                                            .splineToSplineHeading(secondSubDrop, Math.toRadians(-150), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 300))

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

            //        // TODO position failsafes (ramming if position is blocked)
            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);
            if (Bot.vectorDiff(drive.pose.position, secondSubDrop.component1()) > 18) {
                Actions.runBlocking(
                        new ActionHelpersJava.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
                                        bot.actionBucketToStorage(),
                                        drive.actionBuilder(bot.storedPosition)
                                                .stopAndAdd(bot.actionBucketToStorage())
                                                .strafeTo(new Vector2d(secondSub.component1().x-4, secondSub.component1().y))
                                                .stopAndAdd(bot.actionSubPark())
                                                .build()
                                )
                        ));
            }

        } else {
            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    backToSubPark,
                                    new SleepAction(5)
                            )
                    )
            );
        }
    }
}
