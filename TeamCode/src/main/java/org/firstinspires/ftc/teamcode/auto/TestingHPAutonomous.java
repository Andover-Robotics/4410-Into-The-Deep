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

                .splineTo(new Vector2d(13, 27), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(13, 27, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-39.5, 24.5), Math.toRadians(-90))//-41

                .splineToConstantHeading(new Vector2d(-43.1,10), Math.toRadians(126))//140
                .strafeToConstantHeading(new Vector2d(-44,57.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-58,14),Math.toRadians(0))//180
                .strafeToConstantHeading(new Vector2d(-58,55.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToConstantHeading(new Vector2d(-35,14),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-64,10), Math.toRadians(0))//-75
                .afterTime(0.001, bot.actionMoreWallIntakeOpen())

                .strafeToConstantHeading(new Vector2d(-64.5,55), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper()
                ))
//                .strafeToConstantHeading(new Vector2d(-64.5,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 100))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-64.5, 56.5, Math.toRadians(-90)))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(10,58), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 86))
                .splineToConstantHeading(new Vector2d(9,28.2), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 200))
                //.strafeToConstantHeading(new Vector2d(6,29), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 120))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,56), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 123))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(6.5,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(7,28.2), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 85))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,56), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 123))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(3.5,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(4.5,28.2), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 80))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,56), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 125))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .splineToConstantHeading(new Vector2d(5,55), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 110))
                .splineToConstantHeading(new Vector2d(1,28.2), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, 80))

                .afterTime(0.1, bot.actionOpenGripper())

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .setReversed(true)
                .splineTo(new Vector2d(-59,48), Math.toRadians(-10), drive.defaultVelConstraint, new ProfileAccelConstraint(-500,500)) //PARK
//
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