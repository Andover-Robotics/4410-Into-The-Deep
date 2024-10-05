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
    private CRServo leftWheel, rightWheel;

    private static final float maxWheelIncrement = 30;

    public Gripper(OpMode opMode) {
        leftWheel = new CRServo(opMode.hardwareMap, "leftWheel");
        rightWheel = new CRServo(opMode.hardwareMap, "rightWheel");
    }

    public void rotSpecimen(boolean clockwise) {}

    //change these names as necessary thru testing
    public void intake() {
        leftWheel.set(1);
        rightWheel.set(-1);
    }

    public void outtake() {
        leftWheel.set(1);
        rightWheel.set(-1);
    }
}
