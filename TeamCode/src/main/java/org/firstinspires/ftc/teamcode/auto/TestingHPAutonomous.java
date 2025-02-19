package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Testing HP Autonomous", group = "Autonomous")
public class TestingHPAutonomous extends LinearOpMode {
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
        bot.storage();

        Pose2d initialPose = new Pose2d(-9, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action clipAndPreIntake= drive.actionBuilder(drive.pose)
                .stopAndAdd(new SequentialAction(
                        bot.actionHighChamber()
                ))

                .strafeToLinearHeading(new Vector2d(4, 27), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(4, 27, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40.5, 21), Math.toRadians(-90))//-41

                .splineToConstantHeading(new Vector2d(-44.5,10), Math.toRadians(125))//140
                .strafeToConstantHeading(new Vector2d(-44,57.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-58,14),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-58,56.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToConstantHeading(new Vector2d(-40,14),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-64,10), Math.toRadians(0))//-75
                .afterTime(0.01, bot.actionWallIntakeOpen())
                .strafeToConstantHeading(new Vector2d(-64.5,52), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 100))
                .build();

//        Action threeSpecimens = drive.actionBuilder(new Pose2d(-64.5, 52, Math.toRadians(-90)))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//
//                .afterTime(0.01, bot.actionWallToSlidesHighChamber())
//                .strafeTo(new Vector2d(-10, 48), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 150))
//                .splineToConstantHeading(new Vector2d(6,29.5), Math.toRadians(-85), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 140))
//
//                .stopAndAdd(bot.actionSlidesClipDown())
//
//                .afterTime(0.01, bot.actionSlidesClipWall())
//
////                .setReversed(true)
//
////                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
////                .strafeToConstantHeading(new Vector2d(-0,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
//                .strafeToConstantHeading(new Vector2d(-43.5,53.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 150))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//
//                .afterTime(0.01, bot.actionWallToSlidesHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(-18, 50), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 170))
//                .splineToConstantHeading(new Vector2d(5,24), Math.toRadians(-85), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 150))
//
////                .splineToConstantHeading(new Vector2d(5.5,27), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 125))
//
//                .stopAndAdd(bot.actionSlidesClipDown())
//
//                .afterTime(0.01, bot.actionSlidesClipWall())
//
////                .setReversed(true)
//
////                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
////                .strafeToConstantHeading(new Vector2d(5,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
//                .strafeToConstantHeading(new Vector2d(5,42), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
//                .strafeToConstantHeading(new Vector2d(-43.5,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 200))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//
//                .afterTime(0.01, bot.actionWallToSlidesHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(-18, 50), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 170))
//                .splineToConstantHeading(new Vector2d(3.5,24), Math.toRadians(-85), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 200))
//
////                .splineToConstantHeading(new Vector2d(5.5,27), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 125))
//
//                .stopAndAdd(bot.actionSlidesClipDown())
//
//                .afterTime(0.01, bot.actionSlidesClipWall())
//
////                .setReversed(true)
//
////                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
////                .strafeToConstantHeading(new Vector2d(3,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
//                .strafeToConstantHeading(new Vector2d(0,42), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
//                .strafeToConstantHeading(new Vector2d(-43.5,54.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 200))
//
//                .stopAndAdd(new SequentialAction(
//                        bot.actionCloseGripper()
//                ))
//                .afterTime(0.01, bot.actionWallToSlidesHighChamber())
//
//                .strafeToConstantHeading(new Vector2d(-18, 50), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 170))
//                .splineToConstantHeading(new Vector2d(2.5,24), Math.toRadians(-85), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 150))
//
////                .splineToConstantHeading(new Vector2d(5.5,24.5), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 125))
//
//                .stopAndAdd(bot.actionSlidesClipDown())
//
//                .afterTime(0.01, new SequentialAction(
//                        bot.actionSlidesClipStorage()
//                ))
//                .setReversed(true)
//                .splineTo(new Vector2d(-56,51), Math.toRadians(-10), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK
//                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-64.5, 53, Math.toRadians(-90)))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .strafeToConstantHeading(new Vector2d(2, 52), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .strafeToConstantHeading(new Vector2d(2,26.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
                //.strafeToConstantHeading(new Vector2d(6,29), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 120))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(0,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
                .splineToConstantHeading(new Vector2d(0,31), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 85))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(3,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
                .splineToConstantHeading(new Vector2d(3,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 80))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 110))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(6,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 85))
                .splineToConstantHeading(new Vector2d(6,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 80))

                .afterTime(0.1, bot.actionOpenGripper())

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-50,70),Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK

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
                                threeSpecimens
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