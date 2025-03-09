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
    public final double ROLL_MIN = -14;       // Minimum roll value
    public final double ROLL_MAX = 180;     // Maximum roll value
    public final double ROLL_MID = 90;     // Midpoint for straight roll angle
    public final double PITCH_MIN = 0;      // Minimum pitch value (front)
    public final double PITCH_MAX = 270;    // Maximum pitch value (back)
    public final double PITCH_MID = 135;    // Midpoint for vertical pitch angle - straight up through the slides

    public static double rollMultiplier = 1.0663; //integrate into code later
    public static double offset = -5;

    public static double pitchGroundPickup = -93+offset, pitchWallPickup = 189 + offset, pitchFrontPickupToStorage = -55, pitchChamber = 83 + offset, pitchStorage = -100 + offset, pitchOuttakeUp = 80 + offset, pitchOuttakeDown = 70 + offset,
            pitchBucket = 120 + offset, pitchBucketDown = 165 + offset, pitchChamberHoriz = 0 + offset, pitchOuttakeHoriz = 0 + offset, pitchRearChamber = 185;
    public double rollVertical = ROLL_MAX, rollLeft = ROLL_MID, rollTopRight = ROLL_MID - 45, rollTopLeft = ROLL_MID + 45, rollFlipped = ROLL_MIN;

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

    public void rollLeft() {
        if (rollSetpoint > ROLL_MIN) {
            setRoll(rollSetpoint - 45);
        }
    }

    public void rollRight() {
        if (rollSetpoint < ROLL_MAX) {
            setRoll(rollSetpoint + 45);
        }
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

    public void outtakeUp() {
        setRollPitch(rollVertical, pitchOuttakeUp);
    }

//    public void chamberOuttakeUp() {
//        setRollPitch(rollVertical, pitchOuttakeUp-15);
//    }

    public void outtakeDown() {
        setRollPitch(rollVertical, pitchOuttakeDown);
    }

    public void outtakeHoriz() {
        setRollPitch(rollVertical, pitchOuttakeHoriz);
    }

    public void chamberHoriz() {
        setRollPitch(rollVertical, pitchChamberHoriz);
    }

    public void chamber() {
        setRollPitch(rollVertical, pitchChamber);
    }

    public void rearChamber() {
        setRollPitch(rollVertical, pitchRearChamber);
    }

    public void horizLower() {
        setRollPitch(rollVertical, -45);
    }

    public void bucket() {
        setRollPitch(rollVertical, pitchBucket);
    }

    public void bucketDrop() {
        setRollPitch(rollVertical, pitchBucketDown);
    }

    public void wallPickup() {
        setRollPitch(rollFlipped, pitchWallPickup);
    }

    public void frontWallPickup() {
        setRollPitch(rollFlipped, 5);
    }

    public void frontWallUp() {
        setRollPitch(rollFlipped, 40);
    }

    public void higherWallPickup() {
        setRollPitch(rollFlipped, 90);
    }

    public void frontPickup() {
        setRollPitch(rollVertical, pitchGroundPickup);
    }

    public void frontPickup(boolean left, boolean right) {
        double roll = rollVertical;
        if (left && right) {
            roll = rollLeft;
        } else if (left) {
            roll = rollTopLeft;
        } else if (right) {
            roll = rollTopRight;
        }
        setRollPitch(roll, pitchGroundPickup);
    }

    public void pitchPickup() {
        setPitch(pitchGroundPickup);
    }

    public void cv() {
        setRollPitch(rollVertical, pitchGroundPickup-7);
    }

    public void periodic(double pivotAngleDegrees) {
        this.pivotAngleDegrees = pivotAngleDegrees;
        setRollPitch(rollSetpoint, pitchSetpoint);
    }

    // set both roll and pitch at the same time
    public void setRollPitch(double roll, double pitch) {
        pitchSetpoint = pitch;
        rollSetpoint = roll;
        pitch = (pitch % 360) + PITCH_MID - pivotAngleDegrees + 10;
        pitch = pitch % 360;

        // Clamp roll and pitch to their respective ranges
        roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));
        //pitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, pitch));

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
        roll += 10;
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

    // increment roll by a given amount
    public void incrementRoll(double incrementDegrees) {
        setRoll(rollSetpoint + incrementDegrees);

    }
}
