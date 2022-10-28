package org.firstinspires.ftc.teamcode.cv.webcam;

import android.graphics.ImageFormat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

import java.util.concurrent.BlockingQueue;

@Autonomous (name = "TestWebcam", group = "test")
public class TestWebcam extends LinearOpMode {

    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaNavigator.activate(null, webcamName);
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuforiaNavigator.getFrameQueue();

        waitForStart();

        telemetry.addData("RGB565", PIXEL_FORMAT.RGB565);

        int[][] result = VuforiaNavigator.testWebcam(frameQueue);

        while (opModeIsActive() && result == null) {
            result = VuforiaNavigator.testWebcam(frameQueue);
        }

        for (int[] line: result) {
            telemetry.addData("line","fmt %d  w %d  h %d", line[0], line[1], line[2]);
        }
        telemetry.update();

        while (opModeIsActive()) continue;
    }
}
