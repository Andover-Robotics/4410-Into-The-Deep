package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "HP Auto", group = "Autonomous")
public class TestHPAutonomous extends LinearOpMode {
    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        bot.state = Bot.BotState.STORAGE;
        bot.storage();


        // red big init pose NOTE: check comment above each trajectory to find the respective init pose
        Pose2d initialPose = new Pose2d(-10, 63, Math.toRadians(-90));

        Bot.instance = null;
        bot = Bot.getInstance(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action hpAuto = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-15, 46), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13,33),Math.toRadians(-90))
                .waitSeconds(1)

                .strafeToConstantHeading(new Vector2d(-20,35))
                .splineToConstantHeading(new Vector2d(-47,38), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-42,54),Math.toRadians(330))
                .waitSeconds(1)

                //-55 40
                .strafeToLinearHeading(new Vector2d(-58,40), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44,54),Math.toRadians(330))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-6,33), Math.toRadians(-90))
                .strafeTo(new Vector2d(-25,33))

                .strafeToLinearHeading(new Vector2d(-55,25), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44,54),Math.toRadians(120))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-24,13),Math.toRadians(0))
                .build();


        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.periodic(),
                        hpAuto
                )
        );
    }
}
