package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@Config
@TeleOp(name = "cvp2p tester")
public class P2PTester extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    public static double x = 0, y = 0, heading = 90, initHeading = 180;
    public static double x2 = 0, y2 = 0, heading2 = 90;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;

        Pose2d initialPose = new Pose2d(10, 10, Math.toRadians(initHeading));

        boolean left = false;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        bot.openPipeline(false, false, true);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        bot.teleopStorage()
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            bot.autoPeriodic();
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                left = !left;
            }
            if (left) {
                controller.goToPosition(x2, y2, Math.toRadians(heading2), 0.7);
            } else {
                controller.goToPosition(x, y, Math.toRadians(heading), 0.7);
            }
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine("");
            telemetry.addData("first position", left);
            telemetry.addLine("");
            telemetry.addData("x error", controller.getXError());
            telemetry.addData("y error", controller.getYError());
            telemetry.addData("heading error", controller.headingError * 180 / Math.PI);
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                bot.actionDetect(),
                                controller.cvp2p(),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        bot.actionSubAutoPickDown(),
                                        new SleepAction(0.25),
                                        bot.actionSubAutoPickUp(),
                                        new SleepAction(0.3),
                                        bot.actionFrontIntakeToStorage(),
                                        new SleepAction(0.5)

                                ),
                                telemetryPacket -> {
                                    telemetry.addData("sample y val", Bot.sampleYPos);
                                    telemetry.addData("rounds", bot.pipeline.rounds);
                                    telemetry.addData("pose", bot.storedPosition);
                                    telemetry.addData("refAngle", bot.refAngle);
                                    telemetry.addData("target vector", Bot.targetPosition);
                                    telemetry.addData("x error", controller.getXError());
                                    telemetry.addData("y error", controller.getYError());
                                    telemetry.addData("heading error", controller.headingError * 180 / Math.PI);
                                    telemetry.addData("counter", controller.counter);
                                    telemetry.update();
                                    return true;
                                }
                        )
                )
        );
    }
}