package org.firstinspires.ftc.teamcode.cv;

/**
 * Created by JimLori on 11/6/2016.
 */


import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;


/**
 * VuforiaNavigator class contains static methods for navigating using Vuforia. It also contains methods
 * for obtaining images from the video stream.
 *
 * Now updated to allow use of either the built-in phone camera or an external webcam.
 *
 */
public class VuforiaNavigator {

    private static VuforiaLocalizer vuforia;
    private static VuforiaTrackables targets;
    private static String targetAssetName = null;
    private static final OpenGLMatrix DEFAULT_TARGET_LOCATION = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
            AxesOrder.XYX, AngleUnit.DEGREES, 0, 0, 0));
    public static final OpenGLMatrix DEFAULT_CAMERA_LOCATION_ON_ROBOT =
            OpenGLMatrix.translation(0, 0, 0).multiplied(
                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
    private static OpenGLMatrix robotFromPhoneOrCamera = DEFAULT_CAMERA_LOCATION_ON_ROBOT;
    private static final VuforiaLocalizer.CameraDirection DEFAULT_CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;

    /**
     * Create an instance of VuforiaLocalizer, load the Trackables (using default location for each Trackable).
     * For each Trackable, set its phone location and camera direction to default values.
     *
     * @param assetName  Asset name for trackables (null if using only to obtain images)
     * @param webcamName Webcam to use (null for built-in phone camera)
     */
    public static void activate(String assetName, WebcamName webcamName) {
        internalActivate(assetName, null, null, DEFAULT_CAMERA_DIRECTION, webcamName);
    }


    /**
     * Create an instance of VuforiaLocalizer, load the Trackables (setting the location of each Trackable).
     * For each Trackable, set its phone information. Finally, activate the Trackables.
     *
     * @param assetName Asset name for trackables
     * @param targetLocations Array of target locations relative to origin of field coordinate system
     * @param phoneLocation Phone location relative to origin of robot coordinate system
     * @param cameraDirection Phone camera direction (no effect if using external webcam
     * @param webcamName Webcam to use (null for built-in phone camera)
     */
    public static void activate(String assetName, OpenGLMatrix[] targetLocations, OpenGLMatrix phoneLocation,
                                VuforiaLocalizer.CameraDirection cameraDirection, WebcamName webcamName) {
        internalActivate(assetName, targetLocations, phoneLocation, cameraDirection, webcamName);
    }


    /**
     * Create an instance of VuforiaLocalizer, load the Trackables (setting the location of each Trackable).
     * For each Trackable, set its phone information. Finally, activate the Trackables.
     */
    private static void internalActivate(String assetName, OpenGLMatrix[] targetLocations, OpenGLMatrix cameraLocation,
                                         VuforiaLocalizer.CameraDirection cameraDirection, WebcamName webcamName) {
        targetAssetName = assetName;
        if (cameraLocation != null) VuforiaNavigator.robotFromPhoneOrCamera = cameraLocation;

        //Create the VuforiaLocalizer
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.useExtendedTracking = false;
        if(webcamName == null) {
            parameters.cameraDirection = cameraDirection;
        } else {
            parameters.cameraName = webcamName;
        }

        //New key that supports webcams
        parameters.vuforiaLicenseKey = "AUtJ5mz/////AAABmXtx1srReUwRul4uxKENSBMhHVKlyScXHgWkruUT2XRTBrAtPCMF9prCnx7veiSump9zgStSi56lMez7tp8dg56fvi1uO5H2kUkyB2ZUOrPfwWkMKNJ720ldj1nBT1o+svQVIfEE1VSyRqekf7iEvsFpMAtKQ4PQQ2Ud83/lgiOarkzG0oD7RziDERQxN0Dq2Yk1+vF5VqHXrSa8ldxyLknGGH/F3+xHn7GLAtU9hxCGffhZ0yZ92mReYmE/hvcm6JOCxFAXkXgdukIUpiOOAzkAxSMeBQfZjuRMRDo/Ijyy1+cPon08OBIjF/hdz0DSLwq4AYDEW86zpbWCJv8CgZJA4cWqew2Z8zTBZz01i6cE";

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Set up the VuforiaLocalizer to allow frame grabs of RGB565 images
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        //Get targets, set all target locations, and set phone info for each target
        if (assetName == null || assetName.isEmpty()) {
            //If there is no given asset name just return.
            return;
        }
        targets = vuforia.loadTrackablesFromAsset(targetAssetName);
        for (int i = 0; i < targets.size(); i++) {
            if (targetLocations == null) targets.get(i).setLocation(DEFAULT_TARGET_LOCATION);
            else targets.get(i).setLocation(targetLocations[i]);
            if (webcamName == null) {
                ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).setPhoneInformation(robotFromPhoneOrCamera, cameraDirection);
            } else {
                ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).setCameraLocationOnRobot(webcamName, robotFromPhoneOrCamera);
            }
        }

        //activate targets
        targets.activate();

    }

    /**
     * Obtain transform from coordinate system of target i to the FTC Camera coordinate system (the
     * pose of the target relative to the camera).
     * @param i
     * @return
     */
    public static OpenGLMatrix getFtcCameraFromTarget(int i) {
        return ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).getFtcCameraFromTarget();
    }

    /**
     * Obtain transform from the coordinate system of the camera to the coordinate system of target i
     * (the pose of the camera relative to the target).
     * @param i
     * @return
     */
    public static OpenGLMatrix getTargetFromFtcCamera(int i) {
        OpenGLMatrix ftcCameraFromTarget = getFtcCameraFromTarget(i);
        if (ftcCameraFromTarget == null) return null;
        return ftcCameraFromTarget.inverted();
    }

    /**
     * Using target i, obtain transform from the robot coordinate system to the field coordinate
     * system (the pose of the robot on the field). This requires knowledge of the pose of the
     * target on the field and the pose of the camera on the robot.
     * @param i
     * @return
     */
    public static OpenGLMatrix getFieldFromRobot(int i) {
        if (robotFromPhoneOrCamera == null) return null;
        OpenGLMatrix ftcCameraFromRobot = robotFromPhoneOrCamera.inverted();
        OpenGLMatrix ftcFieldFromTarget = targets.get(i).getFtcFieldFromTarget();
        if (ftcFieldFromTarget == null) return null;
        OpenGLMatrix targetFromFtcCamera = getTargetFromFtcCamera(i);
        if (targetFromFtcCamera == null) return null;
        return ftcFieldFromTarget
                .multiplied(targetFromFtcCamera)
                .multiplied(ftcCameraFromRobot);
    }



    /**
     * Set target location on field.
     *
     * @param i              Index of the target.
     * @param targetLocation Target location as OpenGLMatrix (i.e., the target-to-field coordinate transform)
     */
    public static void setTargetLocation(int i, OpenGLMatrix targetLocation) {
        targets.get(i).setLocation(targetLocation);
    }



    /**
     * Provided with a robot-to-field coordinate transform (i.e., a robot pose relative to field),
     * returns the X,Y coordinates and heading Theta of the robot in the field coordinate system.
     * Here, Theta is the angle between the field X axis and the projection of the robot Y axis into
     * the field X-Y plane.
     *
     * @param locationTransform robot-to-field coordinate transform (i.e., robot pose relative to field)
     * @return Array of float containing X,Y, and Theta (radians), in that order
     */
    public static Pose getPoseFromLocationTransform(OpenGLMatrix locationTransform) {
        float[] locationData = locationTransform.getData();
        float x = locationData[12] / 25.4f;
        float y = locationData[13] / 25.4f;
        float theta = (float) Math.atan2(locationData[5], locationData[4]);
        return new Pose(x, y, theta);
    }




    /**
     * returns the frame queue of the VuforiaLocalizer object.
     */
    public static BlockingQueue<VuforiaLocalizer.CloseableFrame> getFrameQueue() {
        return vuforia.getFrameQueue();
    }

    /**
     * Clears the frame queue.
     *
     * @param frameQueue
     */
    public static void clearFrameQueue
    (BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        while (true) {
            tempFrame = frameQueue.poll();
            if (tempFrame == null) break;
            tempFrame.close();
        }
    }

    /**
     * Given the frame queue, and desired image width and height, clears all but the most recent frame
     * in the queue, then searches that frame for an RGB565 image of specified width and height. If
     * such an image is found, loads the destination array with the image pixel values.
     *
     * @param frameQueue The frame queue.
     * @param width      Desired image width.
     * @param height     Desired image height.
     * @param dst        Byte array to hold the RGB565 image pixels, two bytes per pixel, little-endian format.
     * @return true if image was obtained, otherwise false.
     */
    public static boolean getRGB565Array
    (BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int width, int height,
     byte[] dst) {
        if (dst.length != (2 * width * height)) return false;
        VuforiaLocalizer.CloseableFrame frame = null;
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        Image img = null;
        try {
            //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
            while (true) {
                tempFrame = frameQueue.poll();
                if (tempFrame == null) break;
                if (frame != null) frame.close();
                frame = tempFrame;
            }
            if (frame == null) return false;

            //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                img = frame.getImage(i);
                if (img.getFormat() == PIXEL_FORMAT.RGB565 && img.getWidth() == width && img.getHeight() == height) {
                    ByteBuffer byteBuf = img.getPixels();
                    byteBuf.get(dst);
                    return true;
                }
            }
            return false;
        } finally {
            if (frame != null) frame.close();
            if (tempFrame != null) tempFrame.close();
        }
    }

    public static int[][] testWebcam
            (BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
        VuforiaLocalizer.CloseableFrame frame = null;
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        Image img = null;
        try {
            //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
            while (true) {
                tempFrame = frameQueue.poll();
                if (tempFrame == null) break;
                if (frame != null) frame.close();
                frame = tempFrame;
            }
            if (frame == null) return null;


            //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
            long numImages = frame.getNumImages();
            int[][] result = new int[(int)numImages][3];
            for (int i = 0; i < numImages; i++) {
                img = frame.getImage(i);
                result[i][0] = img.getFormat();
                result[i][1] = img.getWidth();
                result[i][2] = img.getHeight();
            }
            return result;
        } finally {
            if (frame != null) frame.close();
            if (tempFrame != null) tempFrame.close();
        }
    }

    /**
     * Turn flashlight on or off
     *
     * @param on true to turn light on, false to turn off.
     * @return true if flashlight was turned on or off as requested, otherwise false.
     */
    public static boolean setFlashTorchMode(boolean on) {
        return CameraDevice.getInstance().setFlashTorchMode(on);

    }

    /**
     * Determine whether specified target is currently visible.
     *
     * @param i Index of target.
     * @return True if target visible (or if position can be inferred with extended tracking), false if not.
     */
    public static boolean isTargetVisible(int i) {
        return ((VuforiaTrackableDefaultListener) targets.get(i).getListener()).isVisible();
    }


//
//    public static int[] getRGBfromByteArray(int row, int col, int width, byte[] src){
//        int index = 2 * (row * width + col);
//        byte b1 = src[index];
//        byte b2 = src[index+1];
//        int blue = (b1 & 0x1F) << 3;
//        int red = (b2 & 0xF8);
//        int green = ((b1 & 0xE0) >> 3) + ((b2 & 0x7) << 5);
//        return new int[] {red, green, blue};
//    }
//
//
//    public static Bitmap getBitmap(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue)
//            throws InterruptedException{
//        VuforiaLocalizer.CloseableFrame frame = null;
//        try{
//            frame = frameQueue.poll(10, TimeUnit.MICROSECONDS);
//            if (frame == null) return null;
//            long numImages = frame.getNumImages();
//            Image rgbImage = null;
//            for (int i = 0; i < numImages; i++)
//                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    rgbImage = frame.getImage(i);
//                    break;
//                }
//            if (rgbImage == null) return null;
//            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(rgbImage.getPixels());
//            return bm;
//        }
//        catch(InterruptedException exc){
//            throw exc;
//        }
//        finally{
//            if (frame != null) frame.close();
//        }
//    }
//

}
