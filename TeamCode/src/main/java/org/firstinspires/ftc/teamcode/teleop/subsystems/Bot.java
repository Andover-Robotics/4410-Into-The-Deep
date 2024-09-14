package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


class Bot {
    public OpMode opMode;
    Motor fl, fr, bl, br;
    private Bot(OpMode opMode) {
        this.opMode = opMode;

        fl = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        fr = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        bl = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        br = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
    }

    public void drive(double strafeSpeed, double straightSpeed, double turnSpeed) {
        double[] speeds = {
                straightSpeed - strafeSpeed - turnSpeed,
                straightSpeed + strafeSpeed + turnSpeed,
                straightSpeed + strafeSpeed - turnSpeed,
                straightSpeed - strafeSpeed + turnSpeed,
        };

        double maxSpeed = 0;

        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }
}