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
public class SpecimenAuto extends LinearOpMode {
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
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(9, 27, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-38, 28), Math.toRadians(-90))//-41

                .splineToConstantHeading(new Vector2d(-37,9), Math.toRadians(110)) // NEW (CAN REMOVE)


                .splineToConstantHeading(new Vector2d(-45,11), Math.toRadians(0))//140
                .strafeToConstantHeading(new Vector2d(-45.1,55.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-56,14),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-56,51.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToLinearHeading(new Pose2d(-46,18, Math.toRadians(-90)),Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-63,10, Math.toRadians(90)), Math.toRadians(0))//0
                .afterTime(0.01, bot.actionFrontWallIntake())

                .strafeToConstantHeading(new Vector2d(-63,62.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))
//                .strafeToConstantHeading(new Vector2d(-64.5,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
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
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-3,36), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

//                .strafeToLinearHeading(new Vector2d(-43,57.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-5,36), drive.defaultVelConstraint, new ProfileAccelConstraint(-80, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipWall())

//                .strafeToLinearHeading(new Vector2d(-43,57.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-43, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
                .strafeToConstantHeading(new Vector2d(-43, 60.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.05)
                ))

                .afterTime(0.01, bot.actionFrontWallToRearSlidesChamber())

                .strafeToConstantHeading(new Vector2d(-7.5,36), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 95))

                .stopAndAdd(new SequentialAction(
                        bot.actionRearSlidesClipDown(),
                        new SleepAction(0.3),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.01, bot.actionRearClipStorage())

                .splineTo(new Vector2d(-62,50), Math.toRadians(165), drive.defaultVelConstraint, new ProfileAccelConstraint(-100,500)) //PARK
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