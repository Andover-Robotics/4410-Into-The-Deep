package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // For Linux/ Mac(i think)?
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueBig = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueBig.runAction(blueBig.getDrive().actionBuilder(new Pose2d(-58, 58, Math.toRadians(-90)))

                //-23, 46
                .strafeToLinearHeading(new Vector2d(-23, 46), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-6,33),Math.toRadians(0))
                        .waitSeconds(1)


                //-55 40
                .strafeToLinearHeading(new Vector2d(-23.5,36), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-36,36), Math.toRadians(-145))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-42,54),Math.toRadians(120))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-58,40), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44,54),Math.toRadians(120))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-55,25), Math.toRadians(-180))
                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-56,47),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-44,54),Math.toRadians(120))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(-36,35), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-24,13, Math.toRadians(0)),Math.toRadians(0))
                .build());



        RoadRunnerBotEntity redBig = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBig.runAction(redBig.getDrive().actionBuilder(new Pose2d(58, -58, Math.toRadians(90)))
                //-23, 46
                .strafeToLinearHeading(new Vector2d(23, -46), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6,-33),Math.toRadians(90))
                .waitSeconds(1)


                //-55 40
                .strafeToLinearHeading(new Vector2d(23.5,-36), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,-36), Math.toRadians(35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(42,-54),Math.toRadians(-60))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(58,-40), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(-60))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(55,-25), Math.toRadians(0))
                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-56,47),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(-60))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(36,-35), Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(24,-13, Math.toRadians(180)),Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBig)
                .addEntity(redBig)
                .start();
    }
}