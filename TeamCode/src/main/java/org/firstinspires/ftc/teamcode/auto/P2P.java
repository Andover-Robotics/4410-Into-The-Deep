package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.signum;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
public class P2P {
    public double integral = 0, lastErrorHeading = 0, lastErrorDrive = 0, lastErrorStrafe = 0;
    MecanumDrive drive;
    public static double hP = 2.2, hI = 0, hD = 3.9;
    public static double dP = -0.15, dD = -0.38;
    public static double sP = -0.46, sD = -0.95;
    public static double dThr = 0.15, sThr = 0.15, tThr = 0.1;
    public double headingError = 0;
    public double inputTurn = 0, driveCorrection = 0, strafeCorrection = 0;
    public Vector2d driveVector;
    public Pose2d target;
    public PoseVelocity2d off = new PoseVelocity2d(new Vector2d(0, 0), 0);
    public int counter = 0, counterMax = 35;

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
        counter++;
        driveVector = new Vector2d(targetX - drive.pose.component1().x, targetY - drive.pose.component1().y);
        double heading = drive.pose.heading.toDouble();

        Vector2d rotatedVector = rotate(driveVector, heading);

        inputTurn = pidHeading(targetH, hP, hI, hD, heading);
        driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.x);
        strafeCorrection = pfdStrafe(sP, sD, 0, rotatedVector.y);

        PoseVelocity2d botVel = new PoseVelocity2d(new Vector2d(driveCorrection, strafeCorrection), inputTurn);
        boolean stopped = (Math.abs(driveCorrection) < dThr) && (Math.abs(strafeCorrection) < sThr) && (Math.abs(inputTurn) < tThr);
        boolean atPos = ((Math.abs(driveVector.x)) < 0.35) && (Math.abs(driveVector.y) < 0.35) && (Math.abs(targetH - heading) < Math.toRadians(2));

        if ((atPos && stopped) || counter > counterMax) {
            drive.setDrivePowers(off);
            return false;
        } else {
            drive.setDrivePowers(botVel);
            return true;
        }
    }

    public boolean roughGoToPosition(double targetX, double targetY, double targetH, double speed, double dispTolerance, double angTolerance) {
        drive.updatePoseEstimate();
        counter++;
        driveVector = new Vector2d(targetX - drive.pose.component1().x, targetY - drive.pose.component1().y);
        double heading = drive.pose.heading.toDouble();

        Vector2d rotatedVector = rotate(driveVector, heading);

        inputTurn = pidHeading(targetH, hP, hI, hD, heading);
        driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.x);
        strafeCorrection = pfdStrafe(sP, sD, 0, rotatedVector.y);

        PoseVelocity2d botVel = new PoseVelocity2d(new Vector2d(driveCorrection, strafeCorrection), inputTurn);
        boolean atPos = ((Math.abs(driveVector.x)) < dispTolerance) && (Math.abs(driveVector.y) < dispTolerance) && (Math.abs(targetH - heading) < Math.toRadians(angTolerance));

        if ((atPos) || counter > counterMax) {
            drive.setDrivePowers(off);
            return false;
        } else {
            drive.setDrivePowers(botVel);
            return true;
        }
    }

    public void demoGoToPosition(double targetX, double targetY, double targetH, double speed) {
        drive.updatePoseEstimate();
        counter++;
        driveVector = new Vector2d(targetX - drive.pose.component1().x, targetY - drive.pose.component1().y);
        double heading = drive.pose.heading.toDouble();

        Vector2d rotatedVector = rotate(driveVector, heading);

        inputTurn = pidHeading(targetH, hP, hI, hD, heading);
        driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.x);
        strafeCorrection = pfdStrafe(sP, sD, 0, rotatedVector.y);

        PoseVelocity2d botVel = new PoseVelocity2d(new Vector2d(driveCorrection, strafeCorrection), inputTurn);
        boolean stopped = (Math.abs(driveCorrection) < dThr) && (Math.abs(strafeCorrection) < sThr) && (Math.abs(inputTurn) < tThr);
        boolean atPos = ((Math.abs(driveVector.x)) < 0.35) && (Math.abs(driveVector.y) < 0.35) && (Math.abs(targetH - heading) < Math.toRadians(2));
        drive.setDrivePowers(botVel);

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

        double newX = -v.x * cosA - v.y * sinA;// +-
        double newY = v.x * sinA - v.y * cosA;// ++
//        double newX = -v.x * cosA - v.y * sinA;// +-


        return new Vector2d(newX, newY);
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public class cvp2p implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return goToPosition(Bot.getTargetPosition().x, Bot.getTargetPosition().y, Bot.getCurrentPosition().heading.toDouble(), 0.7);
        }
    }

    public Action cvp2p() {
        return new cvp2p();
    }

    public class ramp2p implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return roughGoToPosition(target.position.x, target.position.y, target.heading.toDouble(), 1, 3, 90);
        }
    }

    public Action ramp2p(Pose2d newTarget) {
        setTarget(newTarget);
        return new ramp2p();
    }
}
