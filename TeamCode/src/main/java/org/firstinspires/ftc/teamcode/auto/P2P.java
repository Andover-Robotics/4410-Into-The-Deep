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
    public double integral = 0, lastErrorHeading = 0, lastErrorDrive = 0, lastErrorStrafe = 0;
    MecanumDrive drive;
    public static double hP = 2.1, hI = 0, hD = 4.2;
    public static double dP = -0.15, dD = -0.38;
    public static double sP = -0.46, sD = -0.95;
    public double headingError = 0;
    public double inputTurn = 0, driveCorrection = 0, strafeCorrection = 0;
    public Vector2d driveVector;
    public Pose2d target;

    public P2P(MecanumDrive drive) {
        this.drive = drive;
    }

    private double pidHeading(double target, double kp, double ki, double kd, double current) {
        headingError = target - current;
        if (headingError > Math.PI) {
            headingError -= Math.PI*2;
        } else if (headingError < -Math.PI){
            headingError += Math.PI*2;
        }
        integral += headingError;
        double derivative = headingError - lastErrorHeading;
        double correction = (headingError * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = headingError;
        return correction;
    }

    private double pfdDrive(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorDrive;
        double correction = (error * kp) + (derivative * kd);
        correction += signum(error * kf);
        lastErrorDrive = error;
        return correction;
    }

    private double pfdStrafe(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorStrafe;
        double correction = (error * kp) + (derivative * kd);
        correction += signum(error * kf);
        lastErrorStrafe = error;
        return correction;
    }

    public boolean goToPosition(double targetX, double targetY, double targetH, double speed) {
        drive.updatePoseEstimate();
        driveVector = new Vector2d(targetX - drive.pose.component1().x, targetY - drive.pose.component1().y);
        double heading = drive.pose.heading.toDouble();

        Vector2d rotatedVector = rotate(driveVector, heading);

        inputTurn = pidHeading(targetH, hP, hI, hD, heading);
        driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.x);
        strafeCorrection = pfdStrafe(sP, sD, 0, rotatedVector.y);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveCorrection, strafeCorrection), inputTurn));
        return !(((Math.abs(driveVector.x)) < 0.35) && (Math.abs(driveVector.y) < 0.35) && (Math.abs(targetH - heading) < 0.4));
    }

    public double getXError() {
        return driveVector.x;
    }

    public double getYError() {
        return driveVector.y;
    }

    private static Vector2d rotate(Vector2d v, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);

        double newX = -v.x * cosA - v.y * sinA;// +-  0-1, (90), (0), -1+0
        double newY = v.x * sinA - v.y * cosA;// ++  1+0, (90), (0), 0+1

        return new Vector2d(newX, newY);
    }

    public void setTarget(Pose2d target) {
        this.target = target;
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
