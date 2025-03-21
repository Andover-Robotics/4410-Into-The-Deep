package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Lights {
    public OpMode opMode;

    public final Servo lightLeft;
    public final Servo lightRight;

    public enum Color {
        RED,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public enum Light {
        LEFT,
        RIGHT,
    }
    public Lights(OpMode opMode) {
        lightLeft = opMode.hardwareMap.servo.get("lightLeft");
        lightRight = opMode.hardwareMap.servo.get("lightRight");
    }

    public void display (Light light, Color color) {
        switch (light) {
            case LEFT:
                lightLeft.setPosition(colorToPWM(color));
                break;
            case RIGHT:
                lightRight.setPosition(colorToPWM(color));
                break;
        }
    }

    public void disable (Light light, Color color) {
        switch (light) {
            case LEFT:
                lightLeft.setPosition(0);
                break;
            case RIGHT:
                lightRight.setPosition(0);
                break;
        }
    }

    private double colorToPWM(Color color) {
        if (color == Color.WHITE) {
            return 1;
        } else {
            return 0.28 + (0.054 * color.ordinal());
        }
    }
}
