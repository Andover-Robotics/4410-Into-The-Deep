package org.firstinspires.ftc.teamcode.auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.util.SampleDetectionPipeline;

import java.util.concurrent.atomic.AtomicReference;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "tracking tester", group = "Autonomous")
public class TrackingAutonomous extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    double startingY = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        Pose2d initialPose = new Pose2d(20.5, startingY, Math.toRadians(150));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        bot.openPipeline(false, false, true);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                    bot.actionPeriodic(),
                    bot.actionSubAutoIntake()
                )
        );

        while(!isStarted()) {
            bot.autoPeriodic();
            bot.scan();

            gp1.readButtons();
            telemetry.addData("current angle", bot.pipeline.getAngle());
            telemetry.addLine("");
            telemetry.addData("current getx", bot.pipeline.getX());
            telemetry.addData("strafe value", bot.pipeline.getDriveX());
            telemetry.addData("AX value", bot.pipeline.getAX());
            telemetry.addLine("");
            telemetry.addData("slides pixels", bot.pipeline.getYPixels());
            telemetry.addData("current gety", bot.pipeline.getY());
            telemetry.addData("slides value", bot.pipeline.getSlidesY());
            telemetry.addData("area", bot.pipeline.getArea());
            telemetry.addData("aspect ratio", bot.pipeline.getAspectRatio());
            telemetry.addData("density", bot.pipeline.getDensity());
            telemetry.update();
        }

        bot.updateSampleDrive();
        drive.updatePoseEstimate();
        bot.savePosition(drive.pose);

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
//                                bot.actionFrontIntakeToStorage(),
//                                new SleepAction(0.6),
                                bot.actionDetect(),
                                drive.actionBuilderPrecise(bot.storedPosition)
                                        //.stopAndAdd(()-> )
                                        .strafeToConstantHeading(bot.getTargetPosition(), drive.defaultVelConstraint, new ProfileAccelConstraint(SampleDetectionPipeline.decel, SampleDetectionPipeline.accel))
                                        .stopAndAdd(new SequentialAction(
                                                new SleepAction(0.25),
                                                bot.actionSubAutoPickDown(),
                                                new SleepAction(0.25),
                                                bot.actionSubAutoPickUp(),
                                                new SleepAction(0.3),
                                                bot.actionFrontIntakeToStorage(),
                                                new SleepAction(0.5)
                                        ))
                                        .build()
                        ),
                        telemetryPacket -> {
                            telemetry.addData("sample y val", Bot.sampleYPos);
                            telemetry.addData("rounds", bot.pipeline.rounds);
                            telemetry.addData("pose", bot.storedPosition);
                            telemetry.addData("refAngle", bot.refAngle);
                            telemetry.addData("target vector", bot.targetPosition);
                            telemetry.update();
                            return true;
                        }
                )
        );
    }
}