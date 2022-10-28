package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;
@TeleOp(name = "test imu", group = "test")
public class TestIMU extends LinearOpMode {

    BNO055Enhanced imu;

    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle toggleB1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.b;
        }
    };

    public void runOpMode(){
        imu = hardwareMap.get(BNO055Enhanced.class, "imu");

        BNO055Enhanced.AxesMap axesMap = BNO055Enhanced.AxesMap.XYZ;
        BNO055Enhanced.AxesSign axesSign = BNO055Enhanced.AxesSign.PPP;

        while (!opModeIsActive() && !isStopRequested()){

            if (toggleA1.update()){
                axesMap = BNO055Enhanced.AxesMap.values()[(1 + axesMap.ordinal())%6];
            }

            if (toggleB1.update()){
                axesSign = BNO055Enhanced.AxesSign.values()[(1 + axesSign.ordinal())%8];
            }

            telemetry.addData("AxesMap", axesMap);
            telemetry.addData("AxesSign", axesSign);
            telemetry.update();
        }

        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = null;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.axesMap = axesMap;
        parameters.axesSign = axesSign;

        telemetry.addData("WAIT...","");
        telemetry.update();
        imu.initialize(parameters);
        sleep(3000);

        while (opModeIsActive() && ! gamepad1.a) {
            telemetry.addData("system status", imu.getSystemStatus());
            telemetry.addData("calibration status", imu.getCalibrationStatus());
            telemetry.addData("press a to start", "");
            telemetry.update();
        }

        while (opModeIsActive()){

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("AxesMap", axesMap);
            telemetry.addData("AxesSign", axesSign);
            telemetry.addData("Angles", "Hd = %.1f  Pt = %.1f  Rl = %.1f", angles.firstAngle,
                    angles.secondAngle, angles.thirdAngle);
            telemetry.update();
        }

    }

}
