package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // For Linux/ Mac(i think)?
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBig = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redBig.runAction(redBig.getDrive().actionBuilder(new Pose2d(-9, 63, Math.toRadians(-90)))
                //-23, 46
                        .strafeToLinearHeading(new Vector2d(4, 28), Math.toRadians(-90))


//                .strafeToConstantHeading(new Vector2d(-41,42))
//                .strafeToConstantHeading(new Vector2d(-38,10))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-38, 26), Math.toRadians(-90))
                        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-47,12), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-46.9,53))

                //-45,33
                .splineToConstantHeading(new Vector2d(-26,18),Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-56,14),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-56,53))

                .splineToConstantHeading(new Vector2d(-44,16),Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-62,10), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-62.5,53))

                        .splineToConstantHeading(new Vector2d(0,28), Math.toRadians(-80))

                        .setReversed(true)

                        .splineToConstantHeading(new Vector2d(-43,53), Math.toRadians(90))

                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(0,28), Math.toRadians(-80))

                        .setReversed(true)

                        .splineToConstantHeading(new Vector2d(-43,53), Math.toRadians(90))

                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(0,28), Math.toRadians(-80))

                        .setReversed(true)

                        .splineToConstantHeading(new Vector2d(-43,53), Math.toRadians(90))

//
//                .strafeTo(new Vector2d(2,50))
//                .strafeTo(new Vector2d(2,20))
//
//                .strafeTo(new Vector2d(-43,53))
//
//                        .setReversed(false)
//
//                .splineToConstantHeading(new Vector2d(0,20), Math.toRadians(-90))
//                .strafeTo(new Vector2d(-43,53))
//
//
//
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(0,20), Math.toRadians(-90))
//
//
//                .strafeTo(new Vector2d(-43,53))
//
//
//
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(0,20), Math.toRadians(-90))
//
////                .strafeTo(new Vector2d(-50,70)) //PARK
//
//                        .strafeToConstantHeading(new Vector2d(2, 52))
//                        .strafeToConstantHeading(new Vector2d(2,26.5))
//                        //.strafeToConstantHeading(new Vector2d(6,29), drive.defaultVelConstraint, new ProfileAccelConstraint(-110, 120))
//
//                        .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90))
////
////                        .strafeToConstantHeading(new Vector2d(4,65))
//                        .splineToConstantHeading(new Vector2d(0,31), Math.toRadians(-90))
//
//
//                        .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90))
//
//
//                        .strafeToConstantHeading(new Vector2d(0,65))
//                        .splineToConstantHeading(new Vector2d(3,31), Math.toRadians(-90))
//                        .strafeToLinearHeading(new Vector2d(-43,52.5), Math.toRadians(-90))
//
//
//
//                        .strafeToConstantHeading(new Vector2d(5,65))
//                        .splineToConstantHeading(new Vector2d(6,31), Math.toRadians(-90))

                .build()
        );



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueBig)
//                .addEntity(blueSmall)
//                .addEntity(redSmall)
                .addEntity(redBig)
                .start();
    }
}

