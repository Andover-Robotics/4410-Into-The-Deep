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

@Config
@Autonomous(name = "HP Autonomous (zoom zoom)", group = "Autonomous")
public class HPAutonomous extends LinearOpMode {
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

                .strafeToLinearHeading(new Vector2d(-5, 33.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))

//                .splineToLinearHeading(new Pose2d(new Vector2d(-36, 18), 0), -90)
//                .splineToLinearHeading(new Pose2d(new Vector2d(-22, 2), 0), -1)
//                .stopAndAdd(bot.actionSubAutoIntake())
//                .stopAndAdd(new SleepAction(0.6))

                .build();

        Action zoom = drive.actionBuilder(new Pose2d(-5, 34.5, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())
                .strafeToConstantHeading(new Vector2d(-42,42), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                .strafeToConstantHeading(new Vector2d(-38,16), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))
                .splineToConstantHeading(new Vector2d(-47,12), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-47,48.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                //-45,33
                .splineToConstantHeading(new Vector2d(-34,15),Math.toRadians(55))
                .splineToConstantHeading(new Vector2d(-55,14),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-55,48.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, 300))

                .splineToConstantHeading(new Vector2d(-44,16),Math.toRadians(45))
                .afterTime(0.01, bot.actionWallIntakeOpen())
                .splineToConstantHeading(new Vector2d(-64,10), Math.toRadians(-70))
                .strafeToConstantHeading(new Vector2d(-64,50.1), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 80))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-67, 51, Math.toRadians(-90)))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.15)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToConstantHeading(new Vector2d(-14,58), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(-1,35), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 70))
                .strafeToLinearHeading(new Vector2d(0,34), Math.toRadians(-90))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,50.1), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToConstantHeading(new Vector2d(-18,56), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(1,34), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 70))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,50.1), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToConstantHeading(new Vector2d(-18,61), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(3,34), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 70))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,50.1), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToConstantHeading(new Vector2d(-18,51), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(5,34), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 70))

                .afterTime(0.1, bot.actionOpenGripper())

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-34,60),Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-350,90)) //PARK

                .build();

        bot.openPipeline(true, false, false);

        while(!isStarted()) {
            bot.pivot.periodic();

            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {solo = !solo;}
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                redAlliance = !redAlliance;
                bot.pipeline.setBlue(!redAlliance);
                bot.pipeline.setRed(redAlliance);
            }

            telemetry.addData("Solo Auto (Have Both Preloads) (A)", solo);
            if (redAlliance) {
                telemetry.addLine("RED ALLIANCE (B)");
            } else {
                telemetry.addLine("BLUE ALLIANCE (B)");
            }
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                clipAndPreIntake
//                            bot.actionDetectWait()
                        )
                )
        );

        bot.updateSampleDrive();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
//                                bot.actionDetect(),
//                                new SleepAction(0.6),
//                                drive.actionBuilder(new Pose2d(-22, 2, Math.toRadians(0)))
//                                        .strafeToConstantHeading(new Vector2d(-22, 2 - Bot.sampleYPos))
//                                        .stopAndAdd(new SequentialAction(
//                                                bot.actionSubAutoPickDown(),
//                                                new SleepAction(0.3),
//                                                bot.actionSubAutoPickUp(),
//                                                new SleepAction(0.3),
//                                                bot.actionFrontIntakeToStorage(),
//                                                new SleepAction(0.4)
//                                        ))
//                                        .afterTime(0.01, new SequentialAction(
//                                                bot.actionWallIntakeClosed()
//                                        ))
//                                        .strafeToLinearHeading(new Vector2d(-40, 2), Math.toRadians(-90))
//                                        .strafeToLinearHeading(new Vector2d(-44, 47), Math.toRadians(-90))
//                                        .stopAndAdd(bot.actionOpenGripper())
//                                        .stopAndAdd(bot.actionWallIntakeToStorage())
//                                        .strafeToLinearHeading(new Vector2d(-34, 0), Math.toRadians(-90))
//                                        .build(),
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