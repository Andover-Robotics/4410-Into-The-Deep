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

                .splineTo(new Vector2d(9, 27), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionOpenGripper()
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(9, 27, Math.toRadians(-90)))

                .afterTime(0.1, new SequentialAction(
                        bot.actionFirstClipStorage(),
                        bot.actionSecondClipStorage()
                ))

                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-39, 34), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-46,11), Math.toRadians(100))//140
                .strafeToConstantHeading(new Vector2d(-45,55.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToSplineHeading(new Pose2d(-36,14, Math.toRadians(90)),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-56,10),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-56,48))

//                .splineToConstantHeading(new Vector2d(-46,17),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-63.5,11), Math.toRadians(25))//0
                .afterTime(0.01, bot.actionFrontWallIntake())

                .strafeToConstantHeading(new Vector2d(-64.5,62.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-35, 90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))
                .build();

        Action fourSpecimens = drive.actionBuilder(new Pose2d(-64, 62.5, Math.toRadians(90)))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-3,33.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 90))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-4,34), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-6,34), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
//                .strafeToConstantHeading(new Vector2d(-43, 60.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-7.5,34), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 145))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                .strafeToLinearHeading(new Vector2d(-43,60.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))
                .afterTime(0.01, bot.actionFrontWallIntakeToHighBucket())

                .strafeToSplineHeading(new Vector2d(57, 51), Math.toRadians(-135), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 100))

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