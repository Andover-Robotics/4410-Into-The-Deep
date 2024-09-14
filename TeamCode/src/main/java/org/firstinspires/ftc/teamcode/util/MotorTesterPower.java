package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

@TeleOp(name = "Motor Tester Power", group = "Test")
public class MotorTesterPower extends OpMode {
    private static int TP_DELTA = 5;

    private Selector motorSelector;
    private InputColumnResponder input = new InputColumnResponderImpl();
    private DcMotor motor;

    @Override
    public void init() {
        motorSelector = new Selector(hardwareMap.dcMotor.entrySet().stream().map(Map.Entry::getKey));
        input.register(() -> gamepad1.x, motorSelector::selectNext);
    }

    @Override
    public void init_loop() {
        input.update();
        telemetry.addData("Selected motor", motorSelector.selected());
        telemetry.addLine("Press X to select next");
    }

    @Override
    public void start() {
        motor = hardwareMap.dcMotor.get(motorSelector.selected());
        input.clearRegistry();

        input.register(() -> gamepad1.dpad_up, () -> motor.setTargetPosition(motor.getTargetPosition() + TP_DELTA))
                .register(() -> gamepad1.dpad_down, () -> motor.setTargetPosition(motor.getTargetPosition() - TP_DELTA));
    }

    @Override
    public void loop() {
        telemetry.addLine("Controls")
                .addData("Left stick Y", "Controls Power Value");

        telemetry.addLine("Motor Data")
                .addData("Type", motor.getMotorType().getName())
                .addData("Power", "%.3f", motor.getPower())
                .addData("Current Pos", "%d", motor.getCurrentPosition())
                .addData("Target Pos", "%d", motor.getTargetPosition())
                .addData("Busy?", motor.isBusy() ? "Yes" : "No");

        input.update();
        motor.setPower(-gamepad1.left_stick_y);
    }
}
