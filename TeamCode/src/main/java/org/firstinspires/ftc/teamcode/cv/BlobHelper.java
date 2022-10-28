package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

/**
 * Helper class that encapsulates the process of obtaining a new image, cropping and reducing it,
 * converting to binary, and finding blobs.
 */
public class BlobHelper {

    private static final  boolean BLOB_HELPER_LOG = true;
    private static final String BLOB_HELPER_TAG = "BLOB_HELPER";

    private final int IMAGE_WIDTH;
    private final int IMAGE_HEIGHT;
    private final byte[] IMAGE_BYTES;
    private int subRangeX0 = 0;
    private int subRangeY0 = 0;
    private int subRangeWidth;
    private int subRangeHeight;
    private int sampleRatio = 4;
    private int reducedImageWidth;
    private int reducedImageHeight;
    private byte[] reducedImageBytes = null;
    private int[] binaryImage = null;
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = null;


    /**
     * No-arg constructor. Uses defaults of no cropping, sample ratio of 4.
     */
    public BlobHelper(int rawWidth, int rawHeight){
        IMAGE_WIDTH = rawWidth;
        IMAGE_HEIGHT = rawHeight;
        IMAGE_BYTES = new byte[2 * IMAGE_HEIGHT * IMAGE_WIDTH];

        subRangeWidth = IMAGE_WIDTH;
        subRangeHeight = IMAGE_HEIGHT;

        reducedImageWidth = subRangeWidth / sampleRatio;
        reducedImageHeight = subRangeHeight / sampleRatio;


        frameQueue = VuforiaNavigator.getFrameQueue();
        reducedImageBytes = new byte[2 * reducedImageWidth * reducedImageHeight];
        binaryImage = new int[reducedImageWidth * reducedImageHeight];
    }

    /**
     * Constructor
     * @param x0 x-value of upper left corner of subrange (i.e., of cropped image)
     * @param y0 y-value of upper left corner of subrange (i.e., of cropped image)
     * @param width width of subrange
     * @param height height of subrange
     * @param sampleRatio Sample ratio to be used for sampling subrange.
     */
    public BlobHelper(int rawWidth, int rawHeight, int x0, int y0, int width, int height, int sampleRatio){
        IMAGE_WIDTH = rawWidth;
        IMAGE_HEIGHT = rawHeight;
        IMAGE_BYTES = new byte[2 * IMAGE_HEIGHT * IMAGE_WIDTH];

        if (x0 < 0 || y0 < 0 || x0+width > IMAGE_WIDTH || y0+height > IMAGE_HEIGHT)
            throw new IllegalArgumentException("Invalid subrange");
        if (width % sampleRatio != 0 || height % sampleRatio != 0)
            throw new IllegalArgumentException("sampleRatio not a common divisor of width and height");
        frameQueue = VuforiaNavigator.getFrameQueue();
        subRangeX0 = x0;
        subRangeY0 = y0;
        subRangeWidth = width;
        subRangeHeight = height;
        this.sampleRatio = sampleRatio;
        reducedImageWidth = subRangeWidth / sampleRatio;
        reducedImageHeight = subRangeHeight / sampleRatio;
        reducedImageBytes = new byte[2 * reducedImageWidth * reducedImageHeight];
        binaryImage = new int[reducedImageWidth * reducedImageHeight];
    }

    /**
     * Update current image, and obtain the reduced image.
     * @return true if successful (i.e., if new image is available)
     */
    public boolean updateImage(){
        if (!VuforiaNavigator.getRGB565Array(frameQueue, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_BYTES)) return false;
        ImgProc.getReducedRangeRGB565(IMAGE_BYTES, IMAGE_WIDTH, IMAGE_HEIGHT, subRangeX0, subRangeY0,
                subRangeWidth, subRangeHeight, reducedImageBytes, sampleRatio);
        return true;
    }

    /**
     * Find blobs in the reduced image.
     * @param hsvRange Range of HSV for blob detection.
     * @return List of blobs in the image.
     */
    public List<Blob> getBlobs(HSV_Range hsvRange){
        ImgProc.getBinaryImage(reducedImageBytes, hsvRange, binaryImage);
        return Blob.findBlobs(binaryImage, reducedImageWidth, reducedImageHeight);
    }

    /**
     * Find blobs in the reduced image, with filtering.
     * @param hsvRange Range of HSV for blob detection.
     * @param filter Filter to apply to list of blobs before returning.
     * @return List of blobs, filtered.
     */
    public List<Blob> getBlobs(HSV_Range hsvRange, Predicate<Blob> filter){
        ImgProc.getBinaryImage(reducedImageBytes, hsvRange, binaryImage);
        ArrayList<Blob> blobs = Blob.findBlobs(binaryImage, reducedImageWidth, reducedImageHeight);
        for (int i = blobs.size() - 1; i >= 0; i--){
            if (!filter.test(blobs.get(i))) blobs.remove(i);
        }
        return blobs;
    }
        //test
    /**
     * Set new subRange (and sampleRatio) for image reduction.
     * @param x0 x-value of upper left corner of subrange (i.e., of cropped image)
     * @param y0 y-value of upper left corner of subrange (i.e., of cropped image)
     * @param width width of subrange
     * @param height height of subrange
     * @param sampleRatio Sample ratio to be used for sampling subrange.
     */
    public void setSubRange(int x0, int y0, int width, int height, int sampleRatio){
        if (x0 < 0 || y0 < 0 || x0+width > IMAGE_WIDTH || y0+height > IMAGE_HEIGHT)
            throw new IllegalArgumentException("Invalid subrange");
        if (width % sampleRatio != 0 || height % sampleRatio != 0)
            throw new IllegalArgumentException("sampleRatio not a common divisor of width and height");
        subRangeX0 = x0;
        subRangeY0 = y0;
        subRangeWidth = width;
        subRangeHeight = height;
        this.sampleRatio = sampleRatio;
        reducedImageWidth = subRangeWidth / sampleRatio;
        reducedImageHeight = subRangeHeight / sampleRatio;
        reducedImageBytes = new byte[2 * reducedImageWidth * reducedImageHeight];
        binaryImage = new int[subRangeWidth * subRangeHeight];
        ImgProc.getReducedRangeRGB565(IMAGE_BYTES, IMAGE_WIDTH, IMAGE_HEIGHT, subRangeX0, subRangeY0,
                subRangeWidth, subRangeHeight, reducedImageBytes, sampleRatio);
    }

    public boolean saveRawImageFile(String path){
        return ImageSaver.saveImageFile(IMAGE_BYTES, IMAGE_WIDTH, IMAGE_HEIGHT, path);
    }

    public boolean saveReducedImageFile(String path){
        return ImageSaver.saveImageFile(reducedImageBytes, reducedImageWidth, reducedImageHeight, path);
    }

    public boolean saveRawImageFile(boolean dateStamp){
        return ImageSaver.saveImageFile(IMAGE_BYTES, IMAGE_WIDTH, IMAGE_HEIGHT, dateStamp);
    }

    public boolean saveReducedImageFile(boolean dateStamp){
        return ImageSaver.saveImageFile(reducedImageBytes, reducedImageWidth, reducedImageHeight, dateStamp);
    }


}
