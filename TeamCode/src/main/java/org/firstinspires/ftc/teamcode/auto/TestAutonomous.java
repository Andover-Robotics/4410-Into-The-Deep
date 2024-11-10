package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
// import org.firstinspires.ftc.teamcode.MecanumDrive; not resolved

@Config
@Autonomous(name = "TEST_AUTONOMOUS", group = "Autonomous")
public class TestAutonomous extends LinearOpMode {
    private Bot bot;
    public class Arm {
        private ServoEx armLeft, armRight;

        public Arm(HardwareMap hardwareMap) {
            armLeft = hardwareMap.get(ServoEx.class, "armLeft");
            armRight = hardwareMap.get(ServoEx.class, "armRight");
        }

        public class storage implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action storage() {
            return new storage();
        }

        public class wallIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action wallIntake() {
            return new wallIntake();
        }

        public class frontPickup implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action frontPickup() {
            return new frontPickup();
        }

        public class bucket implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action bucket() {
            return new bucket();
        }

        public class chamber implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action chamber() {
            return new chamber();
        }

    }
    public class Gripper {
        private ServoEx servo;

        public Gripper(HardwareMap hardwareMap) {
            servo = hardwareMap.get(ServoEx.class, "servo");
        }

        public class open implements Action {

            @Override
            public boolean run(TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action open() {
            return new open();
        }

        public class close implements Action {

            @Override
            public boolean run(TelemetryPacket packet) {
                return false; // placeholder
            }
        }
        public Action close() {
            return new close();
        }
    }
    public class Pivot {
        private DcMotorEx motor;

        public Pivot(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
        }

        public class storage implements Action {

            @Override
            public boolean run(TelemetryPacket packet) {
                return false; // placeholder
            }
        }
    }
    public class Slides {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -58, Math.toRadians(90));

        Bot.instance = null;
        bot = Bot.getInstance(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // new Pose2d(10, 58, Math.toRadians(-90))
        Action blueSmall = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10, 33), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(32,35), Math.toRadians(-90))

                .turnTo(Math.toRadians(-30))
                .strafeToLinearHeading(new Vector2d(53,53), Math.toRadians(45))


                .strafeToLinearHeading(new Vector2d(40,26), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(48,26), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(53,53), Math.toRadians(45))

                .strafeToLinearHeading(new Vector2d(48,26), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(56,26), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(53,53), Math.toRadians(45))

                .strafeToLinearHeading(new Vector2d(48,15), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(24,13, Math.toRadians(180)),Math.toRadians(180))
                .build();

        //new Pose2d(-10, 58, Math.toRadians(-90))
        Action blueBig = drive.actionBuilder(drive.pose)

                //-23, 46
                .strafeToLinearHeading(new Vector2d(-23, 46), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-6,33),Math.toRadians(0))





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

                .strafeToLinearHeading(new Vector2d(-6,33), Math.toRadians(-90))
                .strafeTo(new Vector2d(-25,33))

                .strafeToLinearHeading(new Vector2d(-55,25), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44,54),Math.toRadians(120))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-24,13),Math.toRadians(0))
                .build();

        //new Pose2d(-10, -58, Math.toRadians(90))
        Action redSmall = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-10, -33), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-32,-35), Math.toRadians(90))

                .turnTo(Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.toRadians(-135))


                .strafeToLinearHeading(new Vector2d(-40,-26), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-48,-26), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.toRadians(-135))

                .strafeToLinearHeading(new Vector2d(-48,-26), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-56,-26), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.toRadians(-135))

                .strafeToLinearHeading(new Vector2d(-48,-15), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24,-13, Math.toRadians(0)),Math.toRadians(0))
                .build();

        //new Pose2d(10, -58, Math.toRadians(90))
        Action redBig = drive.actionBuilder(drive.pose)
                //-23, 46
                .strafeToLinearHeading(new Vector2d(23, -46), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6,-33),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(23.5,-36), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36,-36), Math.toRadians(35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(42,-54),Math.toRadians(-60))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(58,-40), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(-60))
                .waitSeconds(1)

                .strafeToLinearHeading(new Vector2d(6,-33), Math.toRadians(90))
                .strafeTo(new Vector2d(25,-33))

                .strafeToLinearHeading(new Vector2d(55,-25), Math.toRadians(0))
                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-56,47),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(-60))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(24,-13),Math.toRadians(180))
                .build();

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        while(!isStarted()) {
            bot.pivot.periodic();
        }

        Actions.runBlocking(
                new ParallelAction(
                        bot.periodic(),
                        redBig
                )
        );
    }
}
