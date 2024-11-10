package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(36, -60, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-24, 24))
                .waitSeconds(2)
                .lineToX(48)
                .waitSeconds(2)
                .lineToX(60)
                .waitSeconds(2)
                .lineToX(72);


        waitForStart();
        Action trajectoryAction = test.build();
        Actions.runBlocking(
                trajectoryAction
        );
    }
}
