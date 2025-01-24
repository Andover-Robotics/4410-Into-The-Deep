package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gripper {
    public OpMode opMode;
    public final Servo claw;

    public static double closed = 0.64, open = 0.87;

    public enum GripperState {
        CLOSED,
        OPEN
    }

    public GripperState state = GripperState.CLOSED;

    public Gripper(OpMode opMode) {
        claw = opMode.hardwareMap.servo.get("claw");;
    }

    public void close() {
        claw.setPosition(closed);
        state = GripperState.CLOSED;
    }

    public void open() {
        claw.setPosition(open);
        state = GripperState.OPEN;
    }
}