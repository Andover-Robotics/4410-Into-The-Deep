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
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Carpet HP Autonomous (zoom zoom)", group = "Autonomous")
public class CarpetHPAutonomous extends LinearOpMode {
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

                .strafeToLinearHeading(new Vector2d(1, 28), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 100))
                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage(),
                        new SleepAction(0.1)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(1, 28, Math.toRadians(-90)))

                .afterTime(0.1, bot.actionSecondClipStorage())
                .strafeToConstantHeading(new Vector2d(-42,42), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 100))
                .strafeToConstantHeading(new Vector2d(-38,16), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 100))
                .splineToConstantHeading(new Vector2d(-47,12), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-47,48.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 100))

                //-45,33
                .splineToConstantHeading(new Vector2d(-34,15),Math.toRadians(55))
                .splineToConstantHeading(new Vector2d(-56,14),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-56,48.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-100, 100))

                .splineToConstantHeading(new Vector2d(-44,16),Math.toRadians(45))
                .afterTime(0.01, bot.actionWallIntakeOpen())
                .splineToConstantHeading(new Vector2d(-63.5,10), Math.toRadians(-70))
                .strafeToConstantHeading(new Vector2d(-64,54), drive.defaultVelConstraint, new ProfileAccelConstraint(-30, 80))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-64, 54, Math.toRadians(-90)))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.15)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .strafeToConstantHeading(new Vector2d(-1, 60), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .strafeToConstantHeading(new Vector2d(-1,27), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 100))
                .strafeToConstantHeading(new Vector2d(6,27), drive.defaultVelConstraint, new ProfileAccelConstraint(-80, 120))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .strafeToConstantHeading(new Vector2d(-10,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(1,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 85))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .strafeToConstantHeading(new Vector2d(-10,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(3,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 70))

                .stopAndAdd(bot.actionOpenGripper())

                .afterTime(0.01, bot.actionClipWall())

                .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 100))

                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.1)
                ))

                .afterTime(0.01, bot.actionWallToHighChamber())

                .strafeToConstantHeading(new Vector2d(-10,65), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 85))
                .splineToConstantHeading(new Vector2d(5,31), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 70))

                .afterTime(0.1, bot.actionOpenGripper())

                .stopAndAdd(new SequentialAction(
                        bot.actionFirstClipStorage()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionSecondClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-50,70),Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-80,500)) //PARK

                .build();

        //bot.openPipeline(true, false, false);

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