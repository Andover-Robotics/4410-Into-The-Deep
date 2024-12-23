//package org.firstinspires.ftc.teamcode.auto;
//
//// RR-specific imports
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.auto.pipelines.ActionHelpersJava;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
//// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved
//
//@Config
//@Autonomous(name = "Testing Autonomous", group = "Autonomous")
//public class pathTestingAuto extends LinearOpMode {
//    private Bot bot;
//    private GamepadEx gp1;
//
//    boolean solo = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Bot.instance = null;
//        bot = Bot.getInstance(this);
//
//        gp1 = new GamepadEx(gamepad1);
//        telemetry.setAutoClear(true);
//
//        bot.state = Bot.BotState.STORAGE;
//        bot.storage();
//
//
//        // red big init pose NOTE: check comment above each trajectory to find the respective init pose
//        Pose2d initialPose = new Pose2d(-12, 63, Math.toRadians(-90));
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        Action basicPark = drive.actionBuilder(initialPose)
//
//                //-55 40
//                .strafeToConstantHeading(new Vector2d(-6,50))
//                .splineToConstantHeading(new Vector2d(-35,50), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-35,16), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-44,16), Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-44,50))
//
//                //-45,33
//                .splineToConstantHeading(new Vector2d(-41,16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-56,16),Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-56,50))
//
//                .splineToConstantHeading(new Vector2d(-56,16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-64,16),Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-63,50))
//                .build();
//
//        while(!isStarted()) {
//            bot.pivot.periodic();
//
//            gp1.readButtons();
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {solo = !solo;}
//
//            telemetry.addData("Solo Auto (Have Both Preloads)", solo);
//            telemetry.update();
//        }
//
//        Actions.runBlocking(
//                new ActionHelpersJava.RaceParallelCommand(
//                        bot.actionPeriodic(),
//                        basicPark
//                )
//        );
//    }
//}
