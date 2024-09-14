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
    private AnalogInput encoderLeftDiff, encoderRightDiff;
    private ServoEx leftDiff, rightDiff;
    private CRServo leftWheel, rightWheel;

    public Gripper(OpMode opMode) {
        leftWheel = new CRServo(opMode.hardwareMap, "leftWheel");
        rightWheel = new CRServo(opMode.hardwareMap, "rightWheel");
        leftDiff = new SimpleServo(opMode.hardwareMap, "leftDiff", 0, 1);
        rightDiff = new SimpleServo(opMode.hardwareMap, "rightDiff", 0, 1);
    }

    ;

    public void rotSpecimenLeft() {
        //insert some sort of code that uses encoder positions to turn specimen left
    }

    public void rotSpecimenRight() {
        //insert some sort of code that uses encoder positions to turn specimen right
    }
}
