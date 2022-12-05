package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.i2c.RawColors;
import org.firstinspires.ftc.teamcode.i2c.RevColorV2;

@Autonomous(name = "TestRevColorV2", group = "test")
public class TestRevColorV2 extends LinearOpMode {

    RevColorV2 cs;

    public static final AMSColorSensor.Gain HARDWARE_GAIN = AMSColorSensor.Gain.GAIN_16;

    public void runOpMode(){
        cs = hardwareMap.get(RevColorV2.class, "color");
        cs.setHardwareGain(HARDWARE_GAIN);

        while (opModeIsActive()){
            RawColors raw = cs.getRawColors();
            telemetry.addData("Raw Colors", "R %d  G %d  B %d", raw.red,
                    raw.green, raw.blue);
            NormalizedRGBA norm = cs.getNormalizedColors();
            int col = norm.toColor();
            telemetry.addData("Norm colors", "R %d  G %d  B %d", Color.red(col),
                    Color.green(col), Color.blue(col));
            telemetry.update();
        }

    }


}
