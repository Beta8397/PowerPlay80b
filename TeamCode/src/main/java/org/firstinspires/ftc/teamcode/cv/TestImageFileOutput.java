package org.firstinspires.ftc.teamcode.cv;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

/**
 * OpMode that captures RGB565 images from the VuforiaNavigator frame queue, and saves the
 * data to a file (appending if file already exists), currently /sdcard/DCIM/ImageBytes.dat
 *
 * Data is stored as follows:
 * Two bytes to store image width (LSB first)
 * Two bytes to store image height (LSB first)
 * The pixel RGB565 data (two bytes per pixel, little-endian format)
 *
 */

@TeleOp(name = "TestImageFileOutput", group = "Test")
//@Disabled
public class TestImageFileOutput extends LoggingLinearOpMode {

    final boolean SAVE_IMAGE_FILE_LOG = true;
    final String SAVE_IMAGE_FILE_TAG = "SAVE_IMAGE_FILE";
    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() { return gamepad1.a;}
    };
    ButtonToggle toggleB = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {return gamepad1.b; }
    };
    ButtonToggle toggleDUp = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {return gamepad1.dpad_up; }
    };
    ButtonToggle toggleDDown = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {return gamepad1.dpad_down; }
    };

    boolean useWebCam = true;
    WebcamName webcamName = null;

    @Override
    public void runLoggingOpMode() {

//        while (!opModeIsActive() && !isStopRequested()) {
//            if (toggleB.update()) useWebCam = !useWebCam;
//            if (useWebCam) telemetry.addData("External Web Cam","");
//            else telemetry.addData("Phone Camera","");
//            telemetry.addData("Press  B to toggle web cam, > to continue...", "");
//            telemetry.update();
//        }

        if (useWebCam) webcamName = hardwareMap.get(WebcamName.class, "webcam");
        if(webcamName == null)
            VuforiaNavigator.activate(null, null, null, VuforiaLocalizer.CameraDirection.BACK, null);
        else
            VuforiaNavigator.activate(null, webcamName);

//        VuforiaNavigator.setFlashTorchMode(true);
        //Automatically determine the resolution of the camera
        float size[] = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        int imgWidth = Math.round(size[0]);
        int imgHeight = Math.round(size[1]);
        int currentZoom = 0;

        BlobHelper blobHelper = new BlobHelper(imgWidth, imgHeight, 0, 0, imgWidth, imgHeight, 1);

        if (useWebCam) telemetry.addData("Using External Web Cam","");
        else telemetry.addData("Using Phone Camera","");
        telemetry.addData("Resolution"," %d x %d", imgWidth, imgHeight);
        telemetry.addData("Press A to Save Image","");
        telemetry.update();

        int numImagesSaved = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (toggleA.update()) {
                blobHelper.updateImage();
                if (blobHelper.saveReducedImageFile(false)) {
                    telemetry.addData("Image File Saved", "");
                    numImagesSaved++;
                }
                else telemetry.addData("Image Capture Failed","");
                if (useWebCam) telemetry.addData("Using External Web Cam","");
                else telemetry.addData("Using Phone Camera","");
                telemetry.addData("Resolution"," %d x %d", imgWidth, imgHeight);
                telemetry.addData("Zoom", " %d", currentZoom);
                telemetry.addData("Number of Images Saved", " %d", numImagesSaved);
                telemetry.addData("Press A to Save Image","");
                telemetry.update();
            }
//            if(toggleDUp.update() && currentZoom < 30) {
//                currentZoom++;
//                CameraDevice.getInstance().setField("zoom", ""+currentZoom);
//            }
//            if(toggleDDown.update() && currentZoom > 0) {
//                currentZoom--;
//                CameraDevice.getInstance().setField("zoom", ""+currentZoom);
//            }
        }

//        VuforiaNavigator.setFlashTorchMode(false);

    }


}
