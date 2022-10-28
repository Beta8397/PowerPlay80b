package org.firstinspires.ftc.teamcode.cv.webcam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 12/22/2018.
 */
@TeleOp(name = "Web Cam Tester", group = "TeleOP")
@Disabled
public class WebcamTester extends LoggingLinearOpMode {
    WebcamName webcamName;

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaNavigator.activate(null, webcamName);

        waitForStart();
        VuforiaLocalizer.CloseableFrame frame = null;
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuforiaNavigator.getFrameQueue();
        Image img = null;
        while (opModeIsActive()) {

           try {
                //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
                while (true) {
                    tempFrame = frameQueue.poll();
                    if (tempFrame == null) break;
                    if (frame != null) frame.close();
                    frame = tempFrame;
                }

                if (frame == null) continue;

                //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
                long numImages = frame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    img = frame.getImage(i);
                    BetaLog.dd("Res ", "W: %d H: %d", img.getWidth(), img.getHeight());
                    telemetry.addData("Res: ","W: %d H: %d F: %d", img.getWidth(), img.getHeight(), img.getFormat());
                    BetaLog.dd("Format: ", "Format: %d", img.getFormat());
                }
            } finally {
                if (frame != null) frame.close();
                if (tempFrame != null) tempFrame.close();
                frame = null;
                tempFrame = null;
            }
            telemetry.update();


        }
    }
}
