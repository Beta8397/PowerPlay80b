package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.CameraField;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 *
 * Op mode which gets and displays some basic camera parameters. Each parameter has two values, one for the
 * x direction (e.g., width), and one for the y direction (e.g., height). These parameters must
 * be used for CryptoNav navigation. The parameters are:
 *
 * fovRad: camera field of view (x and y), in radians; this will be converted to degrees for display.
 *
 * size: camera resolution in pixels (width and height).
 *
 * pp: camera principal point (x,y). This is the position in the image of the center point of the lens.
 *     You'd think it would just be (width/2, height/2), and it is pretty close to that.
 *
 * fl: camera focal length (x,y), in pixels.
 *
 * As a reality check, the "expected X focal length" is also computed: width/(2 * Math.tan(fovRad[0]/2).
 * This should come out pretty close to fl[0]
 *
 *
 */

@Autonomous(name = "GetCameraParameters", group = "Test")
//@Disabled
public class GetCameraParameters extends LoggingLinearOpMode {

    @Override
    public void runLoggingOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, null, null, VuforiaLocalizer.CameraDirection.BACK, webcamName);
        float[] fovRad = CameraDevice.getInstance().getCameraCalibration().getFieldOfViewRads().getData();
        float[] size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        float[] pp = CameraDevice.getInstance().getCameraCalibration().getPrincipalPoint().getData();
        float[] fl = CameraDevice.getInstance().getCameraCalibration().getFocalLength().getData();
        double expectedFL = size[0]/(2.0 * Math.tan(fovRad[0] / 2.0));
        telemetry.addData("FOV Degrees"," x: %.1f degrees  y: %.1f degrees", fovRad[0]*180.0/Math.PI, fovRad[1]*180.0/Math.PI);
        telemetry.addData("Size"," width: %.1f pixels  height: %.1f pixels", size[0], size[1]);
        telemetry.addData("Principal Point"," x: %.1f y: %.1f", pp[0], pp[1]);
        telemetry.addData("Focal Length"," x: %.1f y: %.1f", fl[0], fl[1]);
        telemetry.addData("Expected X Focal Length", " %.1f", expectedFL);

        int numFields = CameraDevice.getInstance().getNumFields();
        BetaLog.dd("CAMERA FIELDS","");
        CameraField cameraField = new CameraField();
        CameraDevice cd = CameraDevice.getInstance();
        String key = null;
        for (int i = 0; i < numFields; i++){
            if (!cd.getCameraField(i, cameraField)) continue;
            key = cameraField.getKey();
            switch (cameraField.getType()){
                case CameraField.DataType.TypeString:
                    BetaLog.d("%d  key = %s  type = %s  value = \"%s\"", i, key, "String", cd.getFieldString(key));
                    break;
                case CameraField.DataType.TypeInt64:
                    long[] longValue = new long[1];
                    if (!cd.getFieldInt64(key, longValue)) break;
                    BetaLog.d("%d  key = %s  type = %s  value = %d", i, key, "long", longValue[0]);
                    break;
                case CameraField.DataType.TypeFloat:
                    float[] floatValue = new float[1];
                    if (!cd.getFieldFloat(key, floatValue)) break;
                    BetaLog.d("%d  key = %s  type = %s  value = %g", i, key, "float", floatValue);
                    break;
                case CameraField.DataType.TypeBool:
                    boolean[] boolValue = new boolean[1];
                    if (!cd.getFieldBool(key, boolValue)) break;
                    String boolString = boolValue[0]? "true" : "false";
                    BetaLog.d("%d  key = %s  type = %s  value = %s", i, key, "boolean", boolString);
                    break;
                case CameraField.DataType.TypeInt64Range:
                    long[] longValues = new long[2];
                    if (!cd.getFieldInt64Range(key, longValues)) break;
                    BetaLog.d("%d  key = %s  type = %s  values = %d  %d", i, key, "range", longValues[0], longValues[1]);
                    break;
                case CameraField.DataType.TypeUnknown:
                    BetaLog.d("%d  DataType Unknown", i);
                    break;
                default:
                    BetaLog.d("Unrecognized Data Type Specifier");
            }
        }

        BlockingQueue<VuforiaLocalizer.CloseableFrame> queue = VuforiaNavigator.getFrameQueue();
        VuforiaLocalizer.CloseableFrame frame = null;

        telemetry.update();

        waitForStart();

//        CameraDevice.getInstance().setField("zoom", "24");

       /*String previewSize = cd.getFieldString("preview-size");
        telemetry.addData("Old preview size", previewSize);
        cd.setField("preview-size", "1280x960");
        previewSize = cd.getFieldString("preview-size");
        telemetry.update();*/

//        cd.setField("opti-zoom", "opti-zoom-on");
//        cd.setField("zoom", "19");

//        telemetry.addData("OptiZoom", "%s", cd.getFieldString("opti-zoom"));
//        telemetry.addData("Zoom", cd.getFieldString("zoom"));
//        fovRad = CameraDevice.getInstance().getCameraCalibration().getFieldOfViewRads().getData();
//        size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
//        pp = CameraDevice.getInstance().getCameraCalibration().getPrincipalPoint().getData();
//        fl = CameraDevice.getInstance().getCameraCalibration().getFocalLength().getData();
//        expectedFL = size[0]/(2.0 * Math.tan(fovRad[0] / 2.0));
//        telemetry.addData("FOV Degrees"," x: %.1f degrees  y: %.1f degrees", fovRad[0]*180.0/Math.PI, fovRad[1]*180.0/Math.PI);
//        telemetry.addData("Size"," width: %.1f pixels  height: %.1f pixels", size[0], size[1]);
//        telemetry.addData("Principal Point"," x: %.1f y: %.1f", pp[0], pp[1]);
//        telemetry.addData("Focal Length"," x: %.1f y: %.1f", fl[0], fl[1]);
//        telemetry.addData("Expected X Focal Length", " %.1f", expectedFL);
//        telemetry.update();

        while (opModeIsActive()) continue;

    }

}
