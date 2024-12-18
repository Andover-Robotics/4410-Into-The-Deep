package org.firstinspires.ftc.teamcode.cv;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@TeleOp(name = "Sample Detector Test", group = "Concept")
public class SampleDetectionTester extends LinearOpMode
{

    private Bot bot;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        GamepadEx gp1;

        gp1 = new GamepadEx(gamepad1);

        Bot.instance = null;
        bot = Bot.getInstance(this);

        bot.stopMotors();
        bot.state = Bot.BotState.STORAGE;
        //bot.frontIntake();

        SampleDetectionPipeline pipeline = new SampleDetectionPipeline(true, false, false);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            gp1.readButtons();
            telemetry.addData("preview on/off", "... Camera Stream\n");
            pipeline.detect();

            telemetry.addData("x", pipeline.getX());
            telemetry.addData("y", pipeline.getY());
            telemetry.addData("angle", pipeline.getAngle());

            telemetry.update();
            //bot.pivot.periodic();
        }
    }
}
