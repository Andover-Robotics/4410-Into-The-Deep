package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "HP Autonomous (zoom zoom)", group = "Autonomous")
public class HPAutonomous extends LinearOpMode {
    private Bot bot;
    private GamepadEx gp1;

    boolean solo = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        telemetry.setAutoClear(true);

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        Pose2d initialPose = new Pose2d(-12, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action clipAndFirstSample = drive.actionBuilder(drive.pose)
                .afterTime(0.1, bot.actionHighChamber())  // First chamber clip
                .strafeToLinearHeading(new Vector2d(-10.5, 36.2), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.2)
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(-10.5, 36.2, Math.toRadians(-90)))

                .afterTime(0.01, bot.actionClipStorage())

                .strafeToConstantHeading(new Vector2d(-36,40), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 70))
                .strafeToConstantHeading(new Vector2d(-36,16), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 70))
                .splineToConstantHeading(new Vector2d(-47,16), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-47,46), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 70))

                //-45,33
                .splineToConstantHeading(new Vector2d(-29,15),Math.toRadians(55))
                .splineToConstantHeading(new Vector2d(-55,14),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-55,49), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 70))

                .splineToConstantHeading(new Vector2d(-44,16),Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-62,16),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-64,52), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 70))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-65, 53, Math.toRadians(-90)))

                //pick up from wall
                .afterTime(0.01, bot.actionWallIntakeOpen())
                .strafeToLinearHeading(new Vector2d(-38,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))
                .waitSeconds(0.4)


                .strafeToLinearHeading(new Vector2d(-38,53), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.15)
                ))

                .afterTime(0.01, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-3,36.8), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.05, new SequentialAction(
                        bot.actionWallIntakeOpen()
                ))
                .strafeToLinearHeading(new Vector2d(-38,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60)) //run it back

                .strafeToLinearHeading(new Vector2d(-38,53), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.2)
                ))

                .afterTime(0.01, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-5,36.8), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.05, new SequentialAction(
                        bot.actionWallIntakeOpen()
                ))
                .strafeToLinearHeading(new Vector2d(-38,50), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60)) //run it back

                .strafeToLinearHeading(new Vector2d(-38,53), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.2)
                ))

                .afterTime(0.01, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-7,36.8), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-40, 60))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.4),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionClipStorage()
                ))


                .strafeToLinearHeading(new Vector2d(-32,60), Math.toRadians(-90), drive.defaultVelConstraint, new ProfileAccelConstraint(-70, 85)) //PARK

                .build();

        while(!isStarted()) {
            bot.pivot.periodic();

            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {solo = !solo;}

            telemetry.addData("Solo Auto (Have Both Preloads)", solo);
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelpersJava.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                            clipAndFirstSample,
                            zoom,
                            threeSpecimens
                        )
                )
        );
    }
}
