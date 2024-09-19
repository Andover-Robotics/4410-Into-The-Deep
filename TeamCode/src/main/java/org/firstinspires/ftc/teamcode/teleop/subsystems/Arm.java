package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Arm {
    public final Servo armLeft, armRight, wrist;

    public static double armBottomOuttake = 0.685, armTopOuttake = 0.688, autoArmBottomOuttake = 0.75, autoArmTopOuttake = 0.74, armStorage = 0.3, armGround = 0.87, armDualPickup = 0.18, armBlock = 0.3;
    public static double wristBottomOuttake = 0.74, wristTopOuttake = 0.69,  wristStorage = 0.018, wristGround = 0.84, wristDualPickup = 0.03, wristBlock = 0.03;


    public Arm(OpMode opMode) {
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight = opMode.hardwareMap.servo.get("armRight");
        armRight.setDirection(Servo.Direction.FORWARD);
        wrist = opMode.hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setArm(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(1 - position + 0.025);
    }

    public void setWrist(double position) {
        wrist.setPosition(position);
    }

    public void armBlock(){
        setArm(armBlock);
        setWrist(wristBlock);
    }

    public void dualOuttake(int pixel) {
        if (pixel == 1){
            setArm(armTopOuttake);
            setWrist(wristTopOuttake);
        } else {
            setArm(armBottomOuttake);
            setWrist(wristBottomOuttake);
        }
    }

    public void autoDualOuttake(int pixel) {
        if (pixel == 1){
            setArm(autoArmTopOuttake);
            setWrist(wristTopOuttake);
        } else {
            setArm(autoArmBottomOuttake);
            setWrist(wristBottomOuttake);
        }
    }

    public void ground() {
        try {
            setArm(armGround);
            Thread.sleep(250);
            setWrist(wristGround);
        } catch (InterruptedException ex) {
            setWrist(wristGround);
            setArm(armGround);
        }
    }

    public void storage() {
        Thread thread = new Thread(() -> {
            try {
                setWrist(wristStorage);
                Thread.sleep(250);
                setArm(armStorage);
            } catch (InterruptedException ex) {
                setWrist(wristStorage);
                setArm(armStorage);
            }
        });
        thread.start();
    }

    public void pickup() {
        setWrist(wristDualPickup);
        setArm(armDualPickup);
    }

    public void fixDown() {
        setWrist(0.014);
        setArm(armDualPickup);
    }

    public void fixStorage() {
        setWrist(0.022);
        setArm(armStorage);
    }

    public void bottomPixel() {
        setWrist(wristDualPickup);
        setArm(armDualPickup);
    }

    public void runArm(double manual) {
        if (manual > 0.1) {
            if (armLeft.getPosition() > 0.62 && armLeft.getPosition() < 0.82) {//limit values are different because it usually overshoots by 0.002, if it was equal it would get stuck
                setArm(armLeft.getPosition() + 0.005);
            }
        } else if (manual < -0.1){
            if (armLeft.getPosition() > 0.63 && armLeft.getPosition() < 0.83) {
                setArm(armLeft.getPosition() - 0.005);
            }
        }
    }

    public void runAngle(double slidePosition) {
        double desiredAngle = 119.999 - Math.toDegrees(Math.asin((Math.sin(60.001) * 268.754 - (slidePosition / 8.558)) / 170.0));
        double newPosition = 0.0033 * desiredAngle;
        if (slidePosition > -1500) {
            setArm(newPosition);

        }
    }

    public double getArmPosition() {
        return armLeft.getPosition();
    }
}
