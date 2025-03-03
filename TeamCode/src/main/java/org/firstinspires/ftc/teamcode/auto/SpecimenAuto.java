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

                .splineTo(new Vector2d(11, 27), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(11, 27, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, 27), Math.toRadians(-90))//-41

                .splineToConstantHeading(new Vector2d(-37,9), Math.toRadians(110)) // NEW (CAN REMOVE)


                .splineToConstantHeading(new Vector2d(-45.5,11), Math.toRadians(0))//140
                .strafeToConstantHeading(new Vector2d(-45.6,55.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-56,14),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-56,53.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToConstantHeading(new Vector2d(-35,14),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-63,10), Math.toRadians(0))//-75
                .afterTime(0.001, bot.actionMoreWallIntakeOpen())

                .setReversed(true)

                .splineTo(new Vector2d(-63,54.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-32, 80))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))
//                .strafeToConstantHeading(new Vector2d(-64.5,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-64.5, 56.5, Math.toRadians(-90)))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(9,65), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 86))//-90
                .splineToConstantHeading(new Vector2d(9,30.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 200))
                //.strafeToConstantHeading(new Vector2d(6,29), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 120))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

//                .strafeToLinearHeading(new Vector2d(-40,55.2), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 123))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 50), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-40, 55.5))

                .waitSeconds(0.1)

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(6.5,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(7,30.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 85))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

//                .strafeToLinearHeading(new Vector2d(-40,55.2), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 120))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 49.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-40, 55.5))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(4,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(5,30.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-400, 80))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

//                .strafeToLinearHeading(new Vector2d(-40,55.2), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 120))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 49.5), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-40, 56))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(4,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(1,29.5), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-250, 80))

                .afterTime(0.1, bot.actionOpenGripper())

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .setReversed(true)
                .splineTo(new Vector2d(-59,48), Math.toRadians(-15), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK
                .strafeTo(new Vector2d(-60,48), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK

                .build();
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//
//                .afterTime(0.01, bot.actionWallToHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(2, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
//                .strafeToConstantHeading(new Vector2d(2,26.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
//                //.strafeToConstantHeading(new Vector2d(6,29), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 120))
//
//                .stopAndAdd(bot.actionOpenGripper())
//
//                .afterTime(0.01, bot.actionClipWall())
//
//                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper(),
//                        new SleepAction(0.1)
//                ))
//
//                .afterTime(0.01, bot.actionWallToHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(4,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
//                .splineToConstantHeading(new Vector2d(0,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 85))
//
//                .stopAndAdd(bot.actionOpenGripper())
//
//                .afterTime(0.01, bot.actionClipWall())
//
//                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 110))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper(),
//                        new SleepAction(0.1)
//                ))
//
//                .afterTime(0.01, bot.actionWallToHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(0,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
//                .splineToConstantHeading(new Vector2d(3,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 80))
//
//                .stopAndAdd(bot.actionOpenGripper())
//
//                .afterTime(0.01, bot.actionClipWall())
//
//                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 110))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper(),
//                        new SleepAction(0.1)
//                ))
//
//                .afterTime(0.01, bot.actionWallToHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(5,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
//                .splineToConstantHeading(new Vector2d(6,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 80))
//
//                .afterTime(0.1, bot.actionOpenGripper())
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionFirstClipStorage()
//                ))
//
//                .afterTime(0.1, new SequentialAction(
//                        bot.actionSecondClipStorage()
//                ))
//
//                .strafeToLinearHeading(new Vector2d(-50,70),Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK
////
//                .build();
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
                                threeSpecimens
                        )
                )
        );
    }
}