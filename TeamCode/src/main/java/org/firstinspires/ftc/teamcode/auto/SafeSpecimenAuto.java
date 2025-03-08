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
@Autonomous(name = "Specimen Auto (5 Spec)", group = "Autonomous")
public class SafeSpecimenAuto extends LinearOpMode {
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

                .afterDisp(25, new SequentialAction(bot.actionPushIntake()))

                .splineToSplineHeading(new Pose2d(-34, 36, Math.toRadians(-130)), Math.toRadians(180))

                .stopAndAdd(new SequentialAction(
                        bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-33, 58), Math.toRadians(160))

                .afterTime(0.01, bot.actionLiftPush())

                .strafeToLinearHeading(new Vector2d(-45, 32), Math.toRadians(-130))

                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-46, 52), Math.toRadians(140))

                .afterTime(0.01, bot.actionLiftPush())

                .strafeToLinearHeading(new Vector2d(-53, 30), Math.toRadians(-140))

                .stopAndAdd(new SequentialAction(bot.actionDropPush(),
                        new SleepAction(0.2)))

                .strafeToLinearHeading(new Vector2d(-53.5, 55.5), Math.toRadians(160))

                .afterTime(0.01, bot.actionFrontWallIntake())

                .strafeToSplineHeading(new Vector2d(-47.5,47.5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47.5,62.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 45))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))
                .build();

        Action fourSpecimens = drive.actionBuilder(new Pose2d(-63, 62, Math.toRadians(90)))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-1,36), drive.defaultVelConstraint, new ProfileAccelConstraint(-55, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

                //.strafeToLinearHeading(new Vector2d(-43,57.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 55))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-3,35.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

//                .strafeToLinearHeading(new Vector2d(-43,57.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 55))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-5,35.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-80, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

//                .strafeToLinearHeading(new Vector2d(-43,57.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 55))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-7.5,35.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 95))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipStorage())

                .splineTo(new Vector2d(-50,53), Math.toRadians(165), drive.defaultVelConstraint, new ProfileAccelConstraint(-100,200)) //PARK
                .build();
        //bot.openPipeline(true, false, false);

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