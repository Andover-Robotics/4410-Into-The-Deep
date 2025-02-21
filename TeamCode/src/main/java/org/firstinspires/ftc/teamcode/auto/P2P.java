package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.signum;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
public class P2P {
    public double integral = 0, lastErrorHeading = 0, lastErrorDrive = 0;
    MecanumDrive drive;
    public static double hP = 2.1, hI = 0, hD = 4.2;
    public static double dP = -0.12, dD = 0.04;
    public static double sP = -0.32, sD = 0.03;

    public P2P(MecanumDrive drive) {
        this.drive = drive;
    }

    public double pidHeading(double target, double kp, double ki, double kd, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;
        if (error > Math.PI) {
            error -= Math.PI*2;
        } else if (error < -Math.PI){
            error += Math.PI*2;
        }
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
        return correction;
    }

    public double pfdDrive(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorDrive;
        double correction = (error * kp) + (derivative * kd);
        correction += signum(error * kf);
        return correction;
    }

    public boolean goToPosition(double targetX, double targetY, double targetH, double speed) {
        drive.updatePoseEstimate();
        Vector2d driveVector = new Vector2d(targetX - drive.pose.component1().x, targetY - drive.pose.component1().y);
        Vector2d rotatedVector = rotate(driveVector, drive.pose.heading.toDouble());

        double inputTurn = pidHeading(targetH, hP, hI, hD, drive.pose.heading.toDouble());
        double driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.x);
        double strafeCorrection = pfdDrive(sP, sD, 0, rotatedVector.y);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveCorrection, strafeCorrection), inputTurn));
        return (((Math.abs(driveVector.x)) < 0.35) && (Math.abs(driveVector.y) < 0.35) && (Math.abs(targetH - drive.pose.heading.toDouble()) < 0.5));
    }

    public static Vector2d rotate(Vector2d v, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);

        double newX = v.x * cosA - v.y * sinA;
        double newY = v.x * sinA + v.y * cosA;

        return new Vector2d(newX, newY);
    }

    public class p2p implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return goToPosition(Bot.getTargetPosition().x, Bot.getTargetPosition().y, drive.pose.heading.toDouble(), 0.7);
        }
    }

    public Action p2p() {
        return new p2p();
    }
}
