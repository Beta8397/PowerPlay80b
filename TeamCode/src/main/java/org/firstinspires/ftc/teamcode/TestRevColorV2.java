package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.i2c.RawColors;
import org.firstinspires.ftc.teamcode.i2c.RevColorV2;

@Autonomous(name = "TestRevColorV2", group = "test")
public class TestRevColorV2 extends LinearOpMode {

    RevColorV2 cs;

    public static final AMSColorSensor.Gain HARDWARE_GAIN = AMSColorSensor.Gain.GAIN_16;

    public void runOpMode(){
        cs = hardwareMap.get(RevColorV2.class, "color");

        cs.setHardwareGain(AMSColorSensor.Gain.GAIN_16);
        cs.setGain(2);

        telemetry.addData("Sensor Class", cs.getClass().getName());
        telemetry.update();

        waitForStart();

        ElapsedTime et = new ElapsedTime();
        double updateMillis = 0;
        float oldRed = 0;
        float oldGreen = 0;
        float oldBlue = 0;


        while (opModeIsActive()){
            NormalizedRGBA rgba = cs.getNormalizedColors();
            int col = rgba.toColor();

            telemetry.addData("Norm RGB", "R %.3f  G %.3f  B %.3f",
                    rgba.red, rgba.green, rgba.blue);
            telemetry.addData("Color", "R %d  G %d  B %d", Color.red(col),
                    Color.green(col), Color.blue(col));

            float[] hsv = new float[3];

            Color.RGBToHSV(Color.red(col), Color.green(col), Color.blue(col), hsv);

            telemetry.addData("HSV", "H %.3f  S %.3f  V %.3f", hsv[0], hsv[1],
                    hsv[2]);

            if (rgba.red != oldRed || rgba.green != oldGreen || rgba.blue != oldBlue){
                updateMillis = et.milliseconds();
                et.reset();
            }

            telemetry.addData("Update Milliseconds", updateMillis);

            telemetry.update();
        }

    }


}
