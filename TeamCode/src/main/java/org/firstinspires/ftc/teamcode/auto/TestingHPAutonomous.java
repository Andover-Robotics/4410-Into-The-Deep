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

        Action zoom = drive.actionBuilder(new Pose2d(4, 28, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())
//                .strafeToConstantHeading(new Vector2d(-41,42), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//                .strafeToConstantHeading(new Vector2d(-38,10), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//                .splineToConstantHeading(new Vector2d(-47,12), Math.toRadians(0))


//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-40, 30), Math.toRadians(-70))
//
//                .splineToConstantHeading(new Vector2d(-47,12), Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(-46.9,53), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//
//                .splineToConstantHeading(new Vector2d(-30,18),Math.toRadians(-100))
//                .splineToConstantHeading(new Vector2d(-56,14),Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(-56,53), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
//
//                .splineToConstantHeading(new Vector2d(-40,16),Math.toRadians(-100))
//                .splineToConstantHeading(new Vector2d(-62,10), Math.toRadians(180), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 80))
//                .afterTime(0.01, bot.actionWallIntakeOpen())
//                .strafeToConstantHeading(new Vector2d(-62.5,53), drive.defaultVelConstraint, new ProfileAccelConstraint(-20, 80))


//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-41, 24), Math.toRadians(-90))
//
//                .splineToConstantHeading(new Vector2d(-47,10), Math.toRadians(180))

//                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(-42, 44))
                .splineToConstantHeading(new Vector2d(-42, 20), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(-46,14), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-46,56.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-58,14),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-58,56), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToConstantHeading(new Vector2d(-40,14),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-63.8,10), Math.toRadians(0))//-75
                .afterTime(0.01, bot.actionWallIntakeOpen())
                .strafeToConstantHeading(new Vector2d(-64,52), drive.defaultVelConstraint, new ProfileAccelConstraint(-25, 100))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-62.5, 52, Math.toRadians(-90)))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))

                .afterTime(0.1, bot.actionWallToSlidesHighChamber())
                .strafeTo(new Vector2d(-10, 48), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 150))
                .splineToConstantHeading(new Vector2d(6,27), Math.toRadians(-85), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 140))

                .stopAndAdd(bot.actionSlidesClipDown())

                .afterTime(0.01, bot.actionSlidesClipWall())

//                .setReversed(true)

//                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
//                .strafeToConstantHeading(new Vector2d(-0,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
                .strafeToConstantHeading(new Vector2d(-43,53.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 150))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToSlidesHighChamber())

                .splineToConstantHeading(new Vector2d(5.5,27), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 130))

                .stopAndAdd(bot.actionSlidesClipDown())

                .afterTime(0.01, bot.actionSlidesClipWall())

//                .setReversed(true)

//                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
//                .strafeToConstantHeading(new Vector2d(5,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
                .strafeToConstantHeading(new Vector2d(-43,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 150))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToSlidesHighChamber())

                .splineToConstantHeading(new Vector2d(5,27), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 130))

                .stopAndAdd(bot.actionSlidesClipDown())

                .afterTime(0.01, bot.actionSlidesClipWall())

//                .setReversed(true)

//                .splineToConstantHeading(new Vector2d(-43,49), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 120))
//                .strafeToConstantHeading(new Vector2d(3,30), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 150))
                .strafeToConstantHeading(new Vector2d(-43,54.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 150))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))
                .afterTime(0.01, bot.actionWallToSlidesHighChamber())

                .splineToConstantHeading(new Vector2d(5,27), Math.toRadians(-95), drive.defaultVelConstraint, new ProfileAccelConstraint(-95, 130))

                .stopAndAdd(bot.actionSlidesClipDown())

                .afterTime(0.1, new SequentialAction(
                        bot.actionSlidesClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-60,56), Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK

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