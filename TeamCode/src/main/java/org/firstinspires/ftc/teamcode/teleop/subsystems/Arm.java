package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Arm {

    public ServoEx armLeft, armRight;
    public final double MIN_ANGLE = 0, MAX_ANGLE = 473.33;

    // Define the range for roll and pitch
    public final double ROLL_MIN = 0;       // Minimum roll value
    public final double ROLL_MAX = 180;     // Maximum roll value
    public final double ROLL_MID = 90;     // Midpoint for straight roll angle
    public final double PITCH_MIN = 0;      // Minimum pitch value (front)
    public final double PITCH_MAX = 270;    // Maximum pitch value (back)
    public final double PITCH_MID = 135;    // Midpoint for vertical pitch angle - straight up through the slides

    public double pitchGroundPickup = -90, pitchWallPickup = 180, pitchFrontPickupToStorage = -45, pitchRearPickupToStorage = -135, pitchStorage = -90, pitchOuttakeUp = 25, pitchOuttakeDown = 0, pitchBucket = 135;
    public double rollVertical = ROLL_MAX, rollLeft = ROLL_MID, rollTopLeft = ROLL_MID - 45, rollFlipped = ROLL_MIN;

    // Track the current angles for pitch and roll (accounts for pivot angle) (is the output sent to servos)
    public double currentPitch = 0;
    public double currentRoll = 0;

    // Track setpoints for pitch and roll (irrelevant of pivot angle)
    public double pitchSetpoint = 0;
    public double rollSetpoint = 0;

    public double pivotAngleDegrees = 0;

    public Arm(OpMode opMode) {
        armLeft = new SimpleServo(opMode.hardwareMap, "armLeft", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        armRight = new SimpleServo(opMode.hardwareMap, "armRight", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        armLeft.setInverted(true);  // Invert one servo for simpler controls :)
    }

    public void vertical() {
        setRollPitch(rollVertical, 90);
    }

    public void storage() {
        setRollPitch(rollVertical, pitchStorage);
    }

    public void frontPickupToStorage() {
        setRollPitch(rollVertical, pitchFrontPickupToStorage);
    }

    public void rearPickupToStorage() {
        setRollPitch(rollVertical, pitchRearPickupToStorage);
    }

    public void outtakeUp() {
        setRollPitch(rollVertical, pitchOuttakeUp);
    }

    public void outtakeDown() {
        setRollPitch(rollVertical, pitchOuttakeDown);
    }

    public void bucket() {
        setRollPitch(rollLeft, pitchBucket);
    }

    public void wallPickup() {
        setRollPitch(rollFlipped, pitchWallPickup);
    }

    public void frontPickup() {
        setRollPitch(rollVertical, pitchGroundPickup);
    }

    public void rearPickup() {
        setRollPitch(rollFlipped, pitchGroundPickup);
    }

    public void periodic(double pivotAngleDegrees, boolean liveIK) {
        this.pivotAngleDegrees = pivotAngleDegrees;
        if (liveIK) setRollPitch(rollSetpoint, pitchSetpoint); //TODO: uncomment to TEST LIVE UPDATING ARM IK
    }

    // set both roll and pitch at the same time
    public void setRollPitch(double roll, double pitch) {
        pitchSetpoint = pitch;
        rollSetpoint = roll;
        pitch = (pitch % 360) + PITCH_MID - pivotAngleDegrees;
        pitch = pitch % 360;

        // Clamp roll and pitch to their respective ranges
        roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));
        pitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, pitch));

        // Calculate servo angles for combined roll and pitch
        double leftAngle = pitch + (roll);
        double rightAngle = pitch + (ROLL_MAX - (roll));
        // differential kinematics here: https://docs.google.com/spreadsheets/d/1JkhiQhE-VoVGqtl1_H_pUl7kZgCqxzRvsZEbwqYRP3o/edit?usp=sharing

        // Set servos to the calculated angles
        armLeft.turnToAngle(leftAngle);
        armRight.turnToAngle(rightAngle);

        // Update the current roll and pitch values
        currentRoll = roll;
        currentPitch = pitch;
    }

    // set roll only
    public void setRoll(double roll) {
        rollSetpoint = roll;
        // Clamp roll to its range
        roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));

        // Maintain the current pitch while setting roll
        double leftAngle = currentPitch + (roll);
        double rightAngle = currentPitch + (ROLL_MAX - (roll));

        // Set the servo angles for roll
        armLeft.turnToAngle(leftAngle);
        armRight.turnToAngle(rightAngle);

        // Update the current roll value
        currentRoll = roll;
    }

    // set pitch only
    public void setPitch(double pitch) {
        pitchSetpoint = pitch;
        pitch = pitch + PITCH_MID - pivotAngleDegrees;
        // Clamp pitch to its range
        pitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, pitch));

        // Maintain the current roll while setting pitch
        double leftAngle = pitch + (currentRoll);
        double rightAngle = pitch + (ROLL_MAX - (currentRoll));

        // Set the servo angles for pitch
        armLeft.turnToAngle(leftAngle);
        armRight.turnToAngle(rightAngle);

        // Update the current pitch value
        currentPitch = pitch;
    }

    // increment pitch by a given amount
    public void incrementPitch(double incrementDegrees) {
        // Calculate the new pitch
        double newPitch = currentPitch + incrementDegrees;

        // Clamp the new pitch to its range
        newPitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, newPitch));

        // Maintain the current roll while setting the new pitch
        double leftAngle = newPitch + currentRoll;
        double rightAngle = newPitch - currentRoll;

        // Adjust the servos by the relative amount
        armLeft.rotateByAngle(newPitch - currentPitch);
        armRight.rotateByAngle(newPitch - currentPitch);

        // Update the current pitch value
        currentPitch = newPitch;
    }

    // increment roll by a given amount
    public void incrementRoll(double incrementDegrees) {
        // Calculate the new roll
        double newRoll = currentRoll + incrementDegrees;

        // Clamp the new roll to its range
        newRoll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, newRoll));

        // Maintain the current pitch while setting the new roll
        double leftAngle = currentPitch + newRoll;
        double rightAngle = currentPitch - newRoll;

        // Adjust the servos by the relative amount
        armLeft.rotateByAngle(newRoll - currentRoll);
        armRight.rotateByAngle(-(newRoll - currentRoll));

        // Update the current roll value
        currentRoll = newRoll;
    }
}
