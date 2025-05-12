package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@Config
@TeleOp(name = "squid2p tester")
public class Squ2PTester extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    public static double x = 0, y = 0, heading = -90, initHeading = -90;
    public static double x2 = 0, y2 = 0, heading2 = -90;
    public static double speed = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(initHeading));

        boolean left = false;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        P2P controller = new P2P(drive);

        bot.openPipeline(false, false, true);

        while (opModeInInit()) {
            bot.autoPeriodic();
        }


        while (opModeIsActive() && !isStopRequested()) {
            bot.autoPeriodic();
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                left = !left;
            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                bot.pivot.pushIntake(true, true);
//                bot.pivot.arm.outtakeHoriz();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                bot.pivot.changeXZ(0, -3, true, false);
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                bot.pivot.changeXZ(0, 3, true, false);
//            }
            if (left) {
                controller.demoSquidGoToPosition(x2, y2, Math.toRadians(heading2), speed);
            } else {
                controller.demoSquidGoToPosition(x, y, Math.toRadians(heading), speed);
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
            telemetry.addData("vectordiff", Bot.vectorDiff(drive.pose.component1(), new Vector2d(x, y)));
            telemetry.update();
        }
    }
}