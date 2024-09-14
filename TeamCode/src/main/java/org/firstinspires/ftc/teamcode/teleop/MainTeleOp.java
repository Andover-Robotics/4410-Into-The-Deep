package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    private GamepadEx gp1, gp2;

    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

    }
}
