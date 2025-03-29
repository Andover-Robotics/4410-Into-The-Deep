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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Specimen Auto (5+1)", group = "Autonomous")
public class ZoomSpecimenAuto extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    boolean solo = false, redAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;
        bot.initialize();

        Pose2d initialPose = new Pose2d(-9, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action clipAndPreIntake= drive.actionBuilder(drive.pose)
                .stopAndAdd(new SequentialAction(
                        bot.actionHighChamber()
                ))

                .splineTo(new Vector2d(6, 33), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-80, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionOpenGripper()
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(6, 33, Math.toRadians(-90)))

                .afterTime(0.05, new SequentialAction(
                        bot.actionFirstClipStorage(),
                        bot.actionSecondClipStorage()
                ))

                .setReversed(true)

                .afterDisp(32, new SequentialAction(bot.actionPushIntake()))

                .splineToSplineHeading(new Pose2d(-38.5, 37, Math.toRadians(-120)), Math.toRadians(-170))

                .stopAndAdd(new SequentialAction(
                        bot.actionDropPush(),
                        new SleepAction(0.25)))

                .strafeToLinearHeading(new Vector2d(-36, 59), Math.toRadians(160), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 105))

                .afterTime(0.01, bot.actionLiftPush())

                .strafeToLinearHeading(new Vector2d(-46, 30), Math.toRadians(-125), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 105))

                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
                        new SleepAction(0.15)))

                .strafeToLinearHeading(new Vector2d(-47, 52), Math.toRadians(140), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 105))
//
//                .afterTime(0.01, bot.actionLiftPush())
////
//
//                .strafeToLinearHeading(new Vector2d(-52, 19.5), Math.toRadians(-165), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 105))
//
//                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
//                        new SleepAction(0.05)))
//
//                .strafeToLinearHeading(new Vector2d(-53, 55), Math.toRadians(142), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 105))
//
//                .afterTime(0.01, bot.actionFrontWallIntake())
//                .strafeToSplineHeading(new Vector2d(-49,49), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-47.5,63, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
//
                .afterTime(0.01, bot.actionFrontWallIntake())
                .splineToLinearHeading(new Pose2d(-62.5, 12.5, Math.toRadians(90)), Math.toRadians(140))
                .strafeToConstantHeading(new Vector2d(-60, 62))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.02)
                ))
                .build();

        Action fourSpecimens = drive.actionBuilder(new Pose2d(-60, 63, Math.toRadians(90)))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-4.5,35.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper(),
                        new InstantAction(() -> bot.pivot.arm.vertical())
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-43,61), Math.toRadians(110), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))


                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.02)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-6,35.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-130, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.30),
                        bot.actionOpenGripper(),
                        new InstantAction(() -> bot.pivot.arm.vertical())
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-43,61), Math.toRadians(110), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.02)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-7.5,35), drive.defaultVelConstraint, new ProfileAccelConstraint(-130, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper(),
                        new InstantAction(() -> bot.pivot.arm.vertical())
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-43,61), Math.toRadians(110), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.02)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-9.25,35), drive.defaultVelConstraint, new ProfileAccelConstraint(-130, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper(),
                        new InstantAction(() -> bot.pivot.arm.vertical())
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-43,61), Math.toRadians(110), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))
                .waitSeconds(0.01)
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))
                .afterTime(0.01, new SequentialAction(
                        new InstantAction(()->bot.pivot.arm.frontWallUp()),
                        new SleepAction(0.2),
                        bot.actionFrontWallIntakeToHighBucket()
                ))
                .setReversed(true)
                .splineTo(new Vector2d(-0, 46), Math.toRadians(0))
                .splineTo(new Pose2d(52.5, 57, Math.toRadians(-135)).component1(), Math.toRadians(45), drive.defaultVelConstraint, new ProfileAccelConstraint(-35, 70))
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//                .afterTime(0.01, bot.actionFrontWallIntakeToHighBucket())
//
//                .strafeToSplineHeading(new Vector2d(53, 53.5), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 70))

                .stopAndAdd(new SequentialAction(
                        bot.actionBucketDrop(),
                        new SleepAction(0.05),
                        bot.actionBucketToStorage()
                ))

                .build();

        while(!isStarted()) {
            bot.autoPeriodic();
            gp1.readButtons();
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                clipAndPreIntake,
                                zoom,
                                fourSpecimens
                        )
                )
        );
    }
}