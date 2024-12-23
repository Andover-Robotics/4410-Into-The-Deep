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

                .strafeToLinearHeading(new Vector2d(-8, 36), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.3)
                ))

//                .splineToLinearHeading(new Pose2d(new Vector2d(-36, 18), 0), -90)
//                .splineToLinearHeading(new Pose2d(new Vector2d(-22, 2), 0), -1)
//                .stopAndAdd(bot.actionSubAutoIntake())
//                .stopAndAdd(new SleepAction(0.6))

                .build();

        Action zoom = drive.actionBuilder(new Pose2d(-8, 36, Math.toRadians(-90)))


                .strafeToLinearHeading(new Vector2d(-37,36), Math.toRadians(-135))
                .stopAndAdd(bot.actionPushIntake())
                .stopAndAdd(new SleepAction(0.5))
                .strafeToLinearHeading(new Vector2d(-37,50), Math.toRadians(-215))

                .stopAndAdd(bot.actionPickUp())

                .strafeToLinearHeading(new Vector2d(-44,36), Math.toRadians(-135))

                .stopAndAdd(new InstantAction(() -> bot.pivot.changeZ(-5)))
                .stopAndAdd(new SleepAction(0.5))
                .strafeToLinearHeading(new Vector2d(-44,52), Math.toRadians(-225))

                .stopAndAdd(bot.actionFrontIntakeToStorage())

//                .afterTime(0.1, bot.actionSecondClipStorage())
//
//                .strafeToLinearHeading(new Vector2d(-36, 40), Math.toRadians(-90))
//
//                .strafeToConstantHeading(new Vector2d(-36,16))
//                .splineToConstantHeading(new Vector2d(-47,16),Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-47,42))
//
//                //-45,33
//                //.splineToConstantHeading(new Vector2d(-29,15),Math.toRadians(55))
//                .splineToConstantHeading(new Vector2d(-57,14),Math.toRadians(-30))
//                .strafeToConstantHeading(new Vector2d(-57,42))

                .splineToLinearHeading(new Pose2d(new Vector2d(-56,16),Math.toRadians(-90)), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-63,16),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-64,52))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-65, 53, Math.toRadians(-90)))

                //pick up from wall
                .afterTime(0.01, bot.actionWallIntakeOpen())

                .strafeToLinearHeading(new Vector2d(-44,51), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.15)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-6,33), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipWall()
                ))
//
//                .afterTime(0.05, new SequentialAction(
//                        bot.actionWallIntakeOpen()
//                ))

                .strafeToLinearHeading(new Vector2d(-44,51), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.2)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-4,33), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipWall()
                ))
//
//                .afterTime(0.05, new SequentialAction(
//                        bot.actionWallIntakeOpen()
//                ))

                .strafeToLinearHeading(new Vector2d(-44,51), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.2)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-2,33), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipWall()
                ))
//
//                .afterTime(0.05, new SequentialAction(
//                        bot.actionWallIntakeOpen()
//                ))

                .strafeToLinearHeading(new Vector2d(-38,51), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.2)
                ))

                .stopAndAdd(bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(0,36), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-32,60), Math.toRadians(-90)) //PARK

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
