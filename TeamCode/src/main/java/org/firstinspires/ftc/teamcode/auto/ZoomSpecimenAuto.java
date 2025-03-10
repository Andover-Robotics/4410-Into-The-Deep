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

                .splineTo(new Vector2d(9, 26.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionOpenGripper()
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(9, 26.5, Math.toRadians(-90)))

                .afterTime(0.1, new SequentialAction(
                        bot.actionFirstClipStorage(),
                        bot.actionSecondClipStorage()
                ))

                .setReversed(true)

                .afterDisp(32, new SequentialAction(bot.actionPushIntake()))

                .splineToSplineHeading(new Pose2d(-34, 36, Math.toRadians(-130)), Math.toRadians(180))

                .stopAndAdd(new SequentialAction(
                        bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-33, 59), Math.toRadians(160), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 80))

                .afterTime(0.01, bot.actionLiftPush())

                .strafeToLinearHeading(new Vector2d(-46, 32), Math.toRadians(-130), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 95))

                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-47, 52), Math.toRadians(140), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 80))

                .afterTime(0.01, bot.actionLiftPush())

                .strafeToLinearHeading(new Vector2d(-53.5, 30), Math.toRadians(-140), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 95))

                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-53, 57), Math.toRadians(165), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 80))

                .afterTime(0.01, bot.actionFrontWallIntake())

                .strafeToSplineHeading(new Vector2d(-47.5,47.5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47.5,63), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 65))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))
                .build();

        Action fourSpecimens = drive.actionBuilder(new Pose2d(-64, 62.5, Math.toRadians(90)))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-4,34), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-4,33.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-6,33), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-8,33), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,61), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

                .waitSeconds(0.01)
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))
                .afterTime(0.01, bot.actionFrontWallIntakeToHighBucket())
                .setReversed(true)
                .setTangent(Math.toRadians(-7))
                .splineToSplineHeading(new Pose2d(52, 54, Math.toRadians(-135)), Math.toRadians(45))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//                .afterTime(0.01, bot.actionFrontWallIntakeToHighBucket())
//
//                .strafeToSplineHeading(new Vector2d(53, 53.5), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 70))

                .stopAndAdd(new SequentialAction(
                        bot.actionBucketDrop(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, new SequentialAction(
                        bot.actionBucketToFrontIntake()
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