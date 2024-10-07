package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Arm {

    public final ServoEx armLeft, armRight;
    public final double MIN_ANGLE = 0, MAX_ANGLE = 473.33;

    // Define the range for roll and pitch
    public final double ROLL_MIN = 0;       // Minimum roll value
    public final double ROLL_MAX = 230;     // Maximum roll value
    public final double ROLL_MID = 115;     // Midpoint for straight roll angle
    public final double PITCH_MIN = 0;      // Minimum pitch value (front)
    public final double PITCH_MAX = 240;    // Maximum pitch value (back)
    public final double PITCH_MID = 135;    // Midpoint for vertical pitch angle - straight up through the slides

    public double pitchPickup = -90, pitchStorage = 90, pitchOuttakeUp = 25, pitchOuttakeDown = 0, pitchBucket = 170;
    public double rollVertical = ROLL_MID, rollLeft = ROLL_MID - 90, rollRight = ROLL_MID + 90, rollBottomLeft = ROLL_MIN, rollBottomRight = ROLL_MAX;

    // Track the current angles for pitch and roll
    public double currentPitch = 0;
    public double currentRoll = 0;

    public double pivotAngleDegrees = 0;

    public Arm(OpMode opMode) {
        armLeft = new SimpleServo(hardwareMap, "armLeft", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        armRight = new SimpleServo(hardwareMap, "armRight", MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        armRight.setInverted(true);  // Invert one servo for simpler controls :)
    }

    public void storage() {
        setRollPitch(rollVertical, pitchStorage);
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

    public void pickup() {
        setRollPitch(rollRight, pitchPickup);
    }

    public void horizontal() {
        setPitch(0);
    }

    public void periodic(double pivotAngleDegrees) {
        this.pivotAngleDegrees = pivotAngleDegrees;
    }

    // set both roll and pitch at the same time
    public void setRollPitch(double roll, double pitch) {
        pitch = pitch + PITCH_MID - pivotAngleDegrees; // account for angle of pivot and slides to go to absolute angle - angle of 0 is horizontal forwards

        // Clamp roll and pitch to their respective ranges
        roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));
        pitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, pitch));

        // Calculate servo angles for combined roll and pitch
        double leftAngle = pitch + roll;
        double rightAngle = pitch - roll;

        // Set servos to the calculated angles
        armLeft.turnToAngle(leftAngle);
        armRight.turnToAngle(rightAngle);

        // Update the current roll and pitch values
        currentRoll = roll;
        currentPitch = pitch;
    }

    // set roll only
    public void setRoll(double roll) {
        // Clamp roll to its range
        roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));

        // Maintain the current pitch while setting roll
        double leftAngle = currentPitch + roll;
        double rightAngle = currentPitch - roll;

        // Set the servo angles for roll
        armLeft.turnToAngle(leftAngle);
        armRight.turnToAngle(rightAngle);

        // Update the current roll value
        currentRoll = roll;
    }

    // set pitch only
    public void setPitch(double pitch) {
        pitch = pitch + PITCH_MID - pivotAngleDegrees; // account for angle of pivot and slides to go to absolute angle - angle of 0 is horizontal forwards

        // Clamp pitch to its range
        pitch = Math.max(PITCH_MIN, Math.min(PITCH_MAX, pitch));

        // Maintain the current roll while setting pitch
        double leftAngle = pitch + currentRoll;
        double rightAngle = pitch - currentRoll;

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
