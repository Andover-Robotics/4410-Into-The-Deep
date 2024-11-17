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
@Autonomous(name = "HP Autonomous", group = "Autonomous")
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


        // red big init pose NOTE: check comment above each trajectory to find the respective init pose
        Pose2d initialPose = new Pose2d(-10, 63, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action clipAndFirstSample = drive.actionBuilder(drive.pose)
                .afterTime(0.1, bot.actionHighChamber())  // First chamber clip
                .strafeToLinearHeading(new Vector2d(-8, 39.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                .afterTime(0.5, bot.actionFrontIntake()) // Pick up first sample

                .strafeToLinearHeading(new Vector2d(-50,45), Math.toRadians(-90))

                .build();

        Action withPartnerFirstSampleDropoff = drive.actionBuilder(new Pose2d(-50, 45, Math.toRadians(-90)))
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.1),
                        bot.actionWallIntakeClosed(),
                        new SleepAction(1.2),
                        bot.actionOpenGripper(),
                        new SleepAction(0.6),
                        bot.actionWallIntakeToStorage(),
                        new SleepAction(0.5)
                ))

                .strafeToLinearHeading(new Vector2d(-58,44), Math.toRadians(-90))

                .build();

        Action soloFirstSampleSpecimenClip = drive.actionBuilder(new Pose2d(-50, 45, Math.toRadians(-90)))
                .stopAndAdd(new SequentialAction(
                        bot.actionPickDown(),
                        new SleepAction(0.3),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.1),
                        bot.actionWallIntakeClosed(),
                        new SleepAction(1.2),
                        bot.actionOpenGripper()
                ))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-50,50.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                //high chamber (partner "preload" in our hp station)
                .strafeToLinearHeading(new Vector2d(-7,38.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-58,44), Math.toRadians(-90))

                .build();


        Action otherSamplesSpecimen = drive.actionBuilder(new Pose2d(-58, 44, Math.toRadians(-90)))
                
                //pick up second sample
                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        new SleepAction(0.5),
                        bot.actionPickDown(),
                        new SleepAction(0.6),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage(),
                        new SleepAction(0.1),
                        bot.actionWallIntakeClosed(),
                        new SleepAction(1.2),
                        bot.actionOpenGripper()
                ))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-59,50.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                //high chamber (first sample)
                .strafeToLinearHeading(new Vector2d(-5,38.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                // Pick up third sample
                .strafeToLinearHeading(new Vector2d(-40,39), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(-52,26), Math.toRadians(-180))
                .stopAndAdd(new SequentialAction(
                        bot.actionFrontIntake(),
                        bot.actionRotateClaw(),
                        new SleepAction(0.4),
                        bot.actionPickDown(),
                        new SleepAction(0.4),
                        bot.actionPickUp(),
                        new SleepAction(0.2),
                        bot.actionFrontIntakeToStorage()
                ))

                // Drop third sample in HP zone
                .strafeToLinearHeading(new Vector2d(-48,44), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionWallIntakeClosed(),
                        new SleepAction(0.5),
                        bot.actionOpenGripper()
                ))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-48,50.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                //high chamber (2nd sample)
                .strafeToLinearHeading(new Vector2d(-3,38.5), Math.toRadians(-90))

                .afterTime(0.3, new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))
                .strafeToLinearHeading(new Vector2d(-8,38.5), Math.toRadians(-90)) //will push all specimen to the right side
                .build();

        Action oneMoreSpecimenPark = drive.actionBuilder(new Pose2d(-8, 38.5, Math.toRadians(-90)))

                //pick up from wall
                .strafeToLinearHeading(new Vector2d(-48,44), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        bot.actionWallIntakeClosed(),
                        new SleepAction(0.5),
                        bot.actionOpenGripper()
                ))

                .strafeToLinearHeading(new Vector2d(-48,50.5), Math.toRadians(-90))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.3),
                        bot.actionCloseGripper(),
                        new SleepAction(0.5),
                        bot.actionHighChamber()
                ))

                //high chamber (3nd sample????? LETS GO IF WE DO THIS WE"RE CRACKED)
                .strafeToLinearHeading(new Vector2d(-3,38.5), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(
                        bot.actionClipDown(),
                        new SleepAction(0.5),
                        bot.actionClipStorage()
                ))

                .strafeToLinearHeading(new Vector2d(-32,60), Math.toRadians(-90)) //PARK

                .build();

        Action basicPark = drive.actionBuilder(new Pose2d(-8, 38.5, Math.toRadians(-90)))

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
                        bot.periodic(),
                        (solo)? soloFirstSampleSpecimenClip : withPartnerFirstSampleDropoff,
                        otherSamplesSpecimen,
                        (solo)? basicPark : oneMoreSpecimenPark
                )
        );
    }
}
