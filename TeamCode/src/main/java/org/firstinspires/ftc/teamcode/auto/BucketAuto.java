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

    public static Pose2d initialPose = new Pose2d(33, 63, Math.toRadians(-90)); //ONE TILE OVER
    public static Vector2d preloadDrop = new Vector2d(58.5, 55.5);

    public static Pose2d firstSample = new Pose2d(48.6, 42, Math.toRadians(-90));
    public static Pose2d secondSample = new Pose2d(60.25, 41.2, Math.toRadians(-90));
    public static Pose2d thirdSample = new Pose2d(56.2, 39.5, Math.toRadians(-45));

    public static Pose2d firstBucket = new Pose2d(57, 57, Math.toRadians(-135));
    public static Pose2d secondBucket = new Pose2d(57, 57.5, Math.toRadians(-135));
    public static Pose2d thirdBucket = new Pose2d(54.5, 55.5, Math.toRadians(-135));

    public static Pose2d firstCVSample = new Pose2d(firstSample.component1().x, firstSample.component1().y + 1.5, Math.toRadians(-90));
    public static Pose2d secondCVSample = new Pose2d(secondSample.component1().x, secondSample.component1().y + 1.5, Math.toRadians(-90));
    public static Pose2d thirdCVSample = new Pose2d(52.5, 24, Math.toRadians(0));

    public static Pose2d firstSub = new Pose2d(22, 8, Math.toRadians(180));
    public static Pose2d secondSub = new Pose2d(21, 5, Math.toRadians(180));
    public static Pose2d firstSubInter = new Pose2d(34, firstSub.component1().y, Math.toRadians(180));
    public static Pose2d secondSubInter = new Pose2d(34, secondSub.component1().y, Math.toRadians(180));
    public static Pose2d firstSubDrop = new Pose2d(55, 55, Math.toRadians(-135));
    public static Pose2d secondSubDrop = new Pose2d(55, 55, Math.toRadians(-135));

    public static Pose2d subDropRam = new Pose2d(48, 7, Math.toRadians(180));
    public static Pose2d unRam = new Pose2d(23, 7, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        bot.state = Bot.BotState.STORAGE;
        bot.initialize();

        int cv = 0;



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        Action preloadSample = drive.actionBuilder(drive.pose)
                .afterTime(0.01, bot.actionHighBucket())
                .strafeToSplineHeading(preloadDrop, Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 110))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.12),
                        bot.actionBucketDrop(),
                        new SleepAction(0.05)
                ))
                .build();

        Action firstSamplePickup = drive.actionBuilderPrecise(new Pose2d(preloadDrop, Math.toRadians(-135)))
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))

                .strafeToLinearHeading(firstSample.component1(), firstSample.component2()) //first intake pos
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.15),
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

                .waitSeconds(0.25)
                .stopAndAdd(bot.actionBucketDrop())
                .build();

        Action secondSamplePickup = drive.actionBuilderPrecise(firstBucket)

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
                ))
                .strafeToLinearHeading(secondSample.component1(), secondSample.component2())

                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.15),
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

                .waitSeconds(0.25)
                .stopAndAdd(bot.actionBucketDrop())

                .waitSeconds(0.15)
                .build();


        Action thirdSamplePickup = drive.actionBuilderPrecise(secondBucket)
                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        bot.actionDiagFrontIntake(),
                        bot.actionRotateClaw(),
                        bot.actionOpenGripper()
                ))

                .strafeToLinearHeading(thirdSample.component1(), thirdSample.component2(), drive.defaultVelConstraint, new ProfileAccelConstraint(-36, 70))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.1),
                        bot.actionPickDown(),
                        new SleepAction(0.2),
                        bot.actionPickUp(),
                        new SleepAction(0.15)
                ))
                .build();

        Action thirdSampleCVPickup = drive.actionBuilder(thirdSample)
                .afterTime(0.01, bot.actionSubAutoIntake())
                .strafeToLinearHeading(thirdCVSample.component1(), thirdCVSample.component2())
                .build();

        Action thirdSampleSetDropToSub = drive.actionBuilder(thirdSample)
                .afterTime(0.01, new SequentialAction(
                        bot.actionIntakeToHighBucket(true)
                ))
                .strafeToLinearHeading(thirdBucket.component1(), thirdBucket.component2())
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.15),
                        bot.actionBucketDrop(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToStorage(),
                        bot.actionSubAutoIntake()
                ))
                .splineTo(firstSubInter.component1(), Math.toRadians(180))
                .splineTo(firstSub.component1(), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, 350))
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
                .splineTo(secondSubInter.component1(), Math.toRadians(180))
                .splineTo(secondSub.component1(), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, 350))

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
                .splineTo(secondSubInter.component1(), Math.toRadians(180))
                .splineTo(new Vector2d(secondSub.component1().x-2, secondSub.component1().y), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 350))
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
                )).splineToSplineHeading(new Pose2d(new Vector2d(secondSub.component1().x + 10, secondSub.component1().y), Math.toRadians(180)), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 350))
                .afterTime(0.1, bot.actionSubPark())
                .splineTo(secondSubInter.component1(), Math.toRadians(180))
                .splineTo(new Vector2d(secondSub.component1().x-2, secondSub.component1().y), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 350))

                .waitSeconds(5)
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

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                cv = (cv == 0)? 1 : 0;
            }

            if (redAlliance) {
                telemetry.addLine("RED ALLIANCE (B)");
            } else {
                telemetry.addLine("BLUE ALLIANCE (B)");
            }
            if (cv == 0) {
                telemetry.addLine("6 Sample (A)");
            } else {
                telemetry.addLine("5 Sample (A)");
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
                                    drive.actionBuilder(bot.storedPosition)
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .strafeToLinearHeading(firstBucket.component1(), firstBucket.component2())
                                            .waitSeconds(0.3)
                                            .stopAndAdd(bot.actionBucketDrop())
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
                                    drive.actionBuilder(bot.storedPosition)
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .strafeToLinearHeading(secondBucket.component1(), secondBucket.component2())
                                            .waitSeconds(0.3)
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
                                    drive.actionBuilder(bot.storedPosition)
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
                                                    bot.actionBucketDrop()
                                            ))

                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionBucketToStorage(),
                                                    bot.actionSubAutoIntake()
                                            ))
                                            .splineTo(firstSubInter.component1(), Math.toRadians(180))
                                            .splineTo(firstSub.component1(), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 350))
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
                        new SleepAction(0.15),
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
                                drive.actionBuilder(bot.storedPosition)
                                        .stopAndAdd(new SequentialAction(
                                                bot.actionSubAutoPickDown(),
                                                new SleepAction(0.25),
                                                bot.actionSubAutoPickUp()
                                        ))
                                        .afterTime(0.01, new SequentialAction(
                                                bot.actionIntakeToHighBucket()
                                        ))
                                        .setReversed(true)
                                        .splineTo(firstSubDrop.component1(), Math.toRadians(45))
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
            do {
                drive.updatePoseEstimate();
                bot.savePosition(drive.pose);
                Actions.runBlocking(
                        new ActionHelper.RaceParallelCommand(
                                bot.actionPeriodic(),
                                new SequentialAction(
//                                            controller.ramp2p(unRam),
//                                            controller.ramp2p(subDropRam)
                                        drive.actionBuilder(bot.storedPosition)
                                                .strafeToLinearHeading(unRam.position, Math.toRadians(180))
                                                .strafeTo(subDropRam.position, drive.defaultVelConstraint, new ProfileAccelConstraint(-300, 300))
                                                .build()
                                )
                        )
                );
                drive.updatePoseEstimate();
                bot.savePosition(drive.pose);
                cv++;
            } while (Bot.vectorDiff(bot.storedPosition.component1(), subDropRam.component1()) > 6);

            drive.updatePoseEstimate();
            bot.savePosition(drive.pose);

            Actions.runBlocking(
                    new ActionHelpersJava.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    drive.actionBuilder(bot.storedPosition)
                                            //
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionHighBucket()
                                            ))
                                            .setReversed(true)
                                            .splineTo(firstSubDrop.component1(), Math.toRadians(45))
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
                                    new SleepAction(0.15),
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
                                    drive.actionBuilder(bot.storedPosition)
                                            .stopAndAdd(new SequentialAction(
                                                    bot.actionSubAutoPickDown(),
                                                    new SleepAction(0.25),
                                                    bot.actionSubAutoPickUp()
                                            ))
                                            .afterTime(0.01, new SequentialAction(
                                                    bot.actionIntakeToHighBucket()
                                            ))
                                            .setReversed(true)
                                            .splineTo(secondSubDrop.component1(), Math.toRadians(45))
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
                                    finalBucket,
                                    new SleepAction(5)
                            )
                    )
            );
        }
    }
}