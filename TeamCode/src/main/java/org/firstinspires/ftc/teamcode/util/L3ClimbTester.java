package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Pivot;

import java.lang.*;

@Config
@TeleOp(name = "Climb Test")
public class L3ClimbTester extends LinearOpMode {
    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private boolean fieldCentric = false;
    private GamepadEx gp;

    public static final String STORAGE = "storage", PREL2 = "pre-level 2 finished", POSTL2 = "level 2 climb finished",
            TRANSFER = "transferred", PREL3 = "pre-level 3 finished", MIDL3 = "mid-level 3", TILTEDL3 = "tilted level 3", BACKTILTEDL3 = "back tilted level 3",
            POSTL3 = "level 3 climb finished !!!";

    public static String state;
    public static double targetX, targetZ;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.instance = null;
        bot = Bot.getInstance(this);

        bot.storage();
        state = STORAGE;

        gp = new GamepadEx(gamepad1);

        while(!isStarted()) {
            bot.pivot.periodic();
            telemetry.addData("Bot State", bot.state);
            telemetry.addData("Pivot Degrees", bot.pivot.getPivotAngleDegrees());
            telemetry.addData("Pivot IK Target Angle", bot.pivot.pivotIKTargetDegrees);
//            telemetry.addData("Pivot Motor Current", bot.pivot.pivotMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Pivot PID", bot.pivot.power - bot.pivot.calculateFeedForward());
            //telemetry.addData("Pivot Manual Power Up", bot.pivot.manualPowerUp);
            //telemetry.addData("Pivot Target (ticks)", bot.pivot.getTarget());
            //telemetry.addData("Pivot Profiler", bot.pivot.getProfilerTarget());
            //telemetry.addData("Pivot Power", bot.pivot.power);
            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()) {
            gp.readButtons();



            // Pre L2 Climb
            if (state.equals(STORAGE)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.arm.outtakeUp();
                            bot.pivot.prel2Climb(true, false);
                            Thread.sleep(150);
                            bot.pivot.prel2Climb(false, true);

                            state = PREL2;
                            targetX = Pivot.prel2ClimbX;
                            targetZ = Pivot.prel2ClimbHeight;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Finish L2 Climb
            if (state.equals(PREL2)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.midl2Climb(false, true);
                            Thread.sleep(150);
                            bot.pivot.postl2Climb(true, false);
                            Thread.sleep(175);
                            bot.pivot.postl2Climb(false, true);
                            Thread.sleep(500);
                            bot.pivot.slides.climbOn();

                            targetX = Pivot.postl2ClimbX;
                            targetZ = Pivot.postl2ClimbHeight;
                            state = POSTL2;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Transfer to stationary hooks
            if (state.equals(POSTL2)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.climbTransfer(true, false);
                            Thread.sleep(200);
                            bot.pivot.slides.climbOff();
                            Thread.sleep(800);
                            bot.pivot.climbTransfer(false, true);
                            Thread.sleep(1000);

                            targetX = Pivot.climbTransferX;
                            targetZ = Pivot.climbTransferHeight;
                            state = TRANSFER;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Prepare for L3 climb - slides behind top bar
            if (state.equals(TRANSFER)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.prel3Climb(true, false);
                            Thread.sleep(1000);
                            bot.pivot.prel3Climb(false, true);
                            Thread.sleep(1000);

                            targetX = Pivot.prel3ClimbX;
                            targetZ = Pivot.prel3ClimbHeight;
                            state = PREL3;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Mid L3 climb - slides hooks above top bar
            if (state.equals(PREL3)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.midl3Climb(true, false);
                            Thread.sleep(1000);
                            bot.pivot.midl3Climb(false, true);
                            Thread.sleep(1000);

                            targetX = Pivot.midl3ClimbX;
                            targetZ = Pivot.midl3ClimbHeight;
                            state = MIDL3;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Tilted L3 climb - tilted robot forwards to get around bottom bar
            if (state.equals(MIDL3)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.tiltedl3Climb(true, false);
                            Thread.sleep(1000);
                            bot.pivot.tiltedl3Climb(false, true);
                            Thread.sleep(1000);

                            targetX = Pivot.tiltedl3ClimbX;
                            targetZ = Pivot.tiltedl3ClimbHeight;
                            state = TILTEDL3;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Back Tilted L3 climb - tilt robot backwards to get above bottom bar
            if (state.equals(TILTEDL3)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.backTiltedl3Climb(true, false);
                            Thread.sleep(1000);
                            bot.pivot.backTiltedl3Climb(false, true);
                            Thread.sleep(1000);

                            targetX = Pivot.backTiltedl3ClimbX;
                            targetZ = Pivot.backTiltedl3ClimbHeight;
                            state = BACKTILTEDL3;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // Post L3 climb
            if (state.equals(TILTEDL3)) {
                if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                    Thread thread = new Thread(() -> {
                        try {
                            bot.pivot.postl3Climb(false, true);
                            Thread.sleep(1000);
                            bot.pivot.postl3Climb(true, false);
                            Thread.sleep(1000);

                            targetX = Pivot.postl3ClimbX;
                            targetZ = Pivot.postl3ClimbHeight;
                            state = POSTL3;
                        } catch (InterruptedException ignored) {}
                    });
                    thread.start();
                }
            }

            // DRIVE
            drive();

            // TELEMETRY
            telemetry.addData("State: ", state);
            telemetry.addData("TargetX: ", targetX);
            telemetry.addData("TargetZ: ", targetZ);
            telemetry.update();
        }
    }

    private void drive() { // Robot centric, drive multiplier default 1
        driveSpeed = driveMultiplier - 0.5 * gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        if (fieldCentric) {
            Vector2d driveVector = new Vector2d(-gp.getLeftX(), -gp.getLeftY()),
                    turnVector = new Vector2d(-gp.getRightX(), 0);
            bot.driveFieldCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        } else {
            Vector2d driveVector = new Vector2d(gp.getLeftX(), -gp.getLeftY()),
                    turnVector = new Vector2d(gp.getRightX(), 0);
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }
}
