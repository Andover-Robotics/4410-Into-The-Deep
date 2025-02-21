package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.util.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.P2P;


@Config
@TeleOp(name = "p2p tester")
public class TestingP2P extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    public static double x = 0, y = 0, heading = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;

        Pose2d initialPose = new Pose2d(20.5, 0, Math.toRadians(180));

        boolean left = true;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    bot.teleopStorage()
                )
        );


//        bot.openPipeline(false, false, true);
//
//        Actions.runBlocking(
//                new ActionHelpersJava.RaceParallelCommand(
//                        bot.actionPeriodic(),
//                        bot.actionSubAutoIntake()
//                )
//        );

//        while(!isStarted()) {
//            bot.autoPeriodic();
//            bot.scan();
//
//            gp1.readButtons();
//            telemetry.addData("current angle", bot.pipeline.getAngle());
//            telemetry.addLine("");
//            telemetry.addData("current getx", bot.pipeline.getX());
//            telemetry.addData("strafe value", bot.pipeline.getDriveX());
//            telemetry.addData("AX value", bot.pipeline.getAX());
//            telemetry.addLine("");
//            telemetry.addData("slides pixels", bot.pipeline.getYPixels());
//            telemetry.addData("current gety", bot.pipeline.getY());
//            telemetry.addData("slides value", bot.pipeline.getSlidesY());
//            telemetry.addData("area", bot.pipeline.getArea());
//            telemetry.addData("aspect ratio", bot.pipeline.getAspectRatio());
//            telemetry.addData("density", bot.pipeline.getDensity());
//            telemetry.update();
//        }

//        bot.updateSampleDrive();
//        drive.updatePoseEstimate();
//        bot.savePosition(drive.pose);
//
//        Actions.runBlocking(
//                new ActionHelpersJava.RaceParallelCommand(
//                        bot.actionPeriodic(),
//                        new SequentialAction(
////                                bot.actionFrontIntakeToStorage(),
////                                new SleepAction(0.6),
//                                bot.actionDetect(),
//                                drive.actionBuilderPrecise(bot.storedPosition)
//                                        .stopAndAdd(controller.p2p())
//                                        .stopAndAdd(new SequentialAction(
//                                                new SleepAction(0.25),
//                                                bot.actionSubAutoPickDown(),
//                                                new SleepAction(0.25),
//                                                bot.actionSubAutoPickUp(),
//                                                new SleepAction(0.3),
//                                                bot.actionFrontIntakeToStorage(),
//                                                new SleepAction(0.5)
//                                        ))
//                                        .build()
//                        ),
//                        telemetryPacket -> {
//                            telemetry.addData("sample y val", Bot.sampleYPos);
//                            telemetry.addData("rounds", bot.pipeline.rounds);
//                            telemetry.addData("pose", bot.storedPosition);
//                            telemetry.addData("refAngle", bot.refAngle);
//                            telemetry.addData("target vector", bot.targetPosition);
//                            telemetry.update();
//                            return true;
//                        }
//                )
//        );
        while(!isStarted()) {
            bot.autoPeriodic();
            gp1.readButtons();
            telemetry.update();
        }

        bot.updateSampleDrive();
        drive.updatePoseEstimate();

        while (opModeIsActive() && !isStopRequested()) {
            bot.autoPeriodic();
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                left = !left;
            }
            if (left) {
                controller.goToPosition(0, 0, Math.toRadians(180), 0.7);
            } else {
                controller.goToPosition(x, y, Math.toRadians(heading), 0.7);
            }
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine("");
            telemetry.addData("left", left);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);
            telemetry.addLine("");
            telemetry.addData("x error", x - drive.pose.position.x);
            telemetry.addData("y error", y - drive.pose.position.y);
            telemetry.addData("heading error", heading - Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}