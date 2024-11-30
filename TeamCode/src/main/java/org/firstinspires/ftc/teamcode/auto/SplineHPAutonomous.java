package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "Zoomy Autonomous", group = "Autonomous")
public class SplineHPAutonomous extends LinearOpMode {
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
                .strafeToLinearHeading(new Vector2d(-10.5, 37), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.2),
                        bot.actionClipStorage()
                ))
                .build();

        Action zoom = drive.actionBuilder(new Pose2d(-10.5, 37, Math.toRadians(-90)))

//                .strafeToConstantHeading(new Vector2d(-36,40))
//                .strafeToConstantHeading(new Vector2d(-36,16))
//                .strafeToConstantHeading(new Vector2d(-46,16))
//                .splineToConstantHeading(new Vector2d(-46,49), Math.toRadians(90))
//
//                //-45,33
//                .splineToConstantHeading(new Vector2d(-44,15), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-52,15), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-58,50), Math.toRadians(90))
//
//                .strafeToConstantHeading(new Vector2d(-56,18))
//                .splineToConstantHeading(new Vector2d(-64,18), Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-65,53))
//                .build();

                .strafeToConstantHeading(new Vector2d(-36,40))
                .strafeToConstantHeading(new Vector2d(-36,16))
                .splineToConstantHeading(new Vector2d(-44,16), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-44,49))

                //-45,33
                .splineToConstantHeading(new Vector2d(-42,16),Math.toRadians(-110))
                .splineToConstantHeading(new Vector2d(-56,16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-56,50), Math.toRadians(90))

                .strafeToConstantHeading(new Vector2d(-56,16))
                .splineToConstantHeading(new Vector2d(-64,16),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-64,53))
                .build();

        Action threeSpecimens = drive.actionBuilder(new Pose2d(-63, 50, Math.toRadians(-90)))

                //pick up from wall
                .afterTime(0.1, bot.actionWallIntakeOpen())
                .strafeToLinearHeading(new Vector2d(-48,48), Math.toRadians(-90))
                .waitSeconds(0.65)


                .strafeToLinearHeading(new Vector2d(-48,52), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.3)
                ))

                .afterTime(0.1, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-3,37), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionClipStorage(),
                        new SleepAction(0.2),
                        bot.actionWallIntakeOpen()
                ))
                .strafeToLinearHeading(new Vector2d(-48,48), Math.toRadians(-90)) //run it back
                .waitSeconds(0.8)

                .strafeToLinearHeading(new Vector2d(-48,52), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.3)
                ))

                .afterTime(0.1, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-5,37), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionClipStorage(),
                        new SleepAction(0.20),
                        bot.actionWallIntakeOpen()
                ))
                .strafeToLinearHeading(new Vector2d(-48,48), Math.toRadians(-90)) //run it back
                .waitSeconds(0.8)


                .strafeToLinearHeading(new Vector2d(-48,52), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionCloseGripper(),
                        new SleepAction(0.3)
                ))

                .afterTime(0.1, bot.actionHighChamber())

                .strafeToLinearHeading(new Vector2d(-9,37), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.25),
                        bot.actionOpenGripper()
                ))

                .afterTime(0.1, new SequentialAction(
                        bot.actionClipStorage()
                ))


                .strafeToLinearHeading(new Vector2d(-32,60), Math.toRadians(-90)) //PARK

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
