package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

import java.util.List;

@TeleOp(name = "TestSignalSleeve", group = "test")
public class TestSignalSleeve extends LinearOpMode {
    HSV_Range hsvRange = new HSV_Range(90, 150, 0.3f, 1.0f, 0.3f, 1.0f);
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);
        BlobHelper blobHelper = new BlobHelper(640, 480, 0, 0,
                640, 480, 2);
        waitForStart();
        while(opModeIsActive()){
            boolean newImage = blobHelper.updateImage();
            if(newImage){
                List<Blob> blobs = blobHelper.getBlobs(hsvRange);
                if(blobs.size() == 0){
                    telemetry.addData("No blobs", "");
                    telemetry.update();
                    continue;
                } else {
                    while(blobs.size() > 1){
                        if(blobs.get(0).getNumPts() > blobs.get(1).getNumPts()){
                            blobs.remove(1);
                        } else{
                            blobs.remove(0);
                        }
                    }
                    Blob biggestBlob = blobs.get(0);
                    telemetry.addData("BLOB", "X = %.0f  Y = %.0f",
                            biggestBlob.getAvgX(), biggestBlob.getAvgY());
                    telemetry.addData("BLOB", "L = %.0f  W = %.0f",
                            biggestBlob.getLength(), biggestBlob.getWidth());
                    telemetry.addData("BLOB", "NmPoints = %d",
                            biggestBlob.getNumPts());
                    telemetry.addData("BLOB", "Angle = %.1f",
                            Math.toDegrees(biggestBlob.getAngle()));
                    telemetry.update();
                }
            }
        }
    }
}
