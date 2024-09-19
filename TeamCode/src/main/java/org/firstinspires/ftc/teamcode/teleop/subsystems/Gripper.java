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
    // yes you need to use axon encoders no absolute servos turn 360 degrees
    private ServoEx leftDiff, rightDiff;
    private AnalogInput encoderLeftWheel, encoderRightWheel;
    private CRServo leftWheel, rightWheel;

    private static final float wheelIncrement = 30;

    public Gripper(OpMode opMode) {
        leftWheel = new CRServo(opMode.hardwareMap, "leftWheel");
        rightWheel = new CRServo(opMode.hardwareMap, "rightWheel");
        leftDiff = new SimpleServo(opMode.hardwareMap, "leftDiff", 0, 1);
        rightDiff = new SimpleServo(opMode.hardwareMap, "rightDiff", 0, 1);
    }

    public void rotSpecimenL() {
        double leftPos = getAngle(encoderLeftWheel.getVoltage());
        double rightPos = getAngle(encoderRightWheel.getVoltage());


    }

    public void rotSpecimenR() {
        //insert some sort of code that uses encoder positions to turn specimen right
        double leftPos = getAngle(encoderLeftWheel.getVoltage());
        double rightPos = getAngle(encoderRightWheel.getVoltage());


    }

    //change these names as necessary thru testing
    public void intake() {
        leftWheel.set(1);
        rightWheel.set(-1);
    }

    public void outtake() {
        leftWheel.set(1);
        rightWheel.set(-1);
    }

    private double getAngle(double encoderValue) {
        return encoderValue/3.3*360;
    }

    private void CrMove(double angle, CRServo crServo, AnalogInput encoder) {
        double currentAngle = getAngle(encoder.getVoltage()), targetAngle;

        if (angle < 0) {
            targetAngle = currentAngle + angle/360;
            crServo.set(-1);
            
        } else {
            crServo.set(1);
        }
    }
}
