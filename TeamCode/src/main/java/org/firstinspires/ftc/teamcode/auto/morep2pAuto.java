package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@Config
@Autonomous(name = "more p2p auto")
public class morep2pAuto extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    public static double x = 0, y = 0, heading = -90, initHeading = -90;
    public static double x2 = 0, y2 = 0, heading2 = -90;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(initHeading));
        Pose2d first = new Pose2d(x, y, Math.toRadians(heading));
        Pose2d second = new Pose2d(x2, y2, Math.toRadians(heading2));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        bot.storage();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic()
                )
        );


        waitForStart();
        drive.updatePoseEstimate();

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                new ParallelAction(
                                        bot.actionFrontWallIntake(),
                                        controller.roughp2p(first)
                                ),
                                new SleepAction(3),
                                new ParallelAction(
                                        bot.actionFrontWallToRearSlidesChamber(),
                                        controller.roughp2p(second)
                                ),
                                new SleepAction(5),
                                new ParallelAction(
                                        bot.actionFrontWallIntake(),
                                        controller.roughp2p(first)
                                ),
                                new SleepAction(3),
                                new ParallelAction(
                                        bot.actionFrontWallToRearSlidesChamber(),
                                        controller.roughp2p(second)
                                )
                        )
                )
        );
    }
}