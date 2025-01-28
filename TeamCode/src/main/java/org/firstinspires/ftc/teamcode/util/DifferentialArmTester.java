package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Arm;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@TeleOp(name = "Diffy Tester", group = "Toolbox")
public class DifferentialArmTester extends LinearOpMode {
    public Bot bot;
    private GamepadEx gp1;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        bot.pivot.arm.vertical();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.pivot.arm.setPitch(bot.pivot.arm.pitchSetpoint - 2.5);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.pivot.arm.setRoll(bot.pivot.arm.rollSetpoint - 2.5);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.pivot.arm.setPitch(bot.pivot.arm.pitchSetpoint + 2.5);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.pivot.arm.setRoll(bot.pivot.arm.rollSetpoint + 2.5);
            }

            telemetry.addData("Right Servo Position", bot.pivot.arm.armRight.getAngle());
            telemetry.addData("Left Servo Position", bot.pivot.arm.armLeft.getAngle());

            telemetry.addData("Current Roll Position (dpad L/R)", bot.pivot.arm.rollSetpoint);
            telemetry.addData("Current Pitch Position (dpad Up/Down)", bot.pivot.arm.pitchSetpoint);

            telemetry.update();
            bot.periodic();
        }
    }

}
