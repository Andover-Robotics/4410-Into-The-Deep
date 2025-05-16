package org.firstinspires.ftc.teamcode.auto.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LConstants {
    static {
      TwoWheelConstants.forwardTicksToInches = 0.001971414489896;
        TwoWheelConstants.strafeTicksToInches = 0.001971414489896;
        TwoWheelConstants.forwardY = -156.0 / 25.4;
        TwoWheelConstants.strafeX = 170.0 / 25.4;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "motorFL";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "motorBR";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(new Orientation(
              AxesReference.INTRINSIC,
              AxesOrder.XYZ,
              AngleUnit.DEGREES,
              90,
              90,
              (float) 42.520819,
              0  // acquisitionTime, not used
      ));

    }
}




