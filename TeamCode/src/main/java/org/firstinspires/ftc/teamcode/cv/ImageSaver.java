package org.firstinspires.ftc.teamcode.cv;


import org.firstinspires.ftc.teamcode.logging.BetaLog;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.Calendar;
import java.util.GregorianCalendar;


/**
 * ImageSaver provides a method to save data from RGB565 images to a file. If file already exists,
 * image data is appended to existing data.
 *
 * Data is stored as follows:
 * Two bytes to store image width (LSB first)
 * Two bytes to store image height (LSB first)
 * The pixel RGB565 data (two bytes per pixel, little-endian format)
 *
 */
public class ImageSaver {

    private static final boolean SAVE_IMAGE_FILE_LOG = true;
    private static final String SAVE_IMAGE_FILE_TAG = "SAVE_IMAGE_FILE";

    /**
     * Saves RGB565 image to a file, with file name specified by path argument.
     * @param imageBytes Byte array containing the RGB565 image, 2 bytes per pixel, little-endian order
     * @param imgWidth Image width
     * @param imgHeight Image height
     * @param path Path for file storage
     * @return true if successful, otherwise false
     */
    public static boolean saveImageFile(byte[] imageBytes, int imgWidth, int imgHeight, String path){

        FileOutputStream fos = null;

        //This awkward nested try/catch/finally block is needed because fos.close() can itself throw an exception

        try {

            try {
                fos = new FileOutputStream(path, true); //"true" means to append if file already exists

                //this array just holds the image dimensions for writing to the file
                byte[] bytes = new byte[]{(byte)(imgWidth&0xFF), (byte)(imgWidth>>8), (byte)(imgHeight&0xFF), (byte)(imgHeight>>8)};

                //Write the image dimensions, then the actual image data to the file
                fos.write(bytes);
                fos.write(imageBytes);
            } catch (FileNotFoundException exc) {
                if (SAVE_IMAGE_FILE_LOG)  BetaLog.dd(SAVE_IMAGE_FILE_TAG, "FileNotFoundException Thrown");
                return false;
            } catch (java.io.IOException exc) {
                if (SAVE_IMAGE_FILE_LOG) BetaLog.dd(SAVE_IMAGE_FILE_TAG, "IOException Thrown");
                return false;
            } finally {
                if (fos != null) fos.close();
                if (SAVE_IMAGE_FILE_LOG) BetaLog.dd(SAVE_IMAGE_FILE_TAG, "ImageFile Saved and Closed");
            }
        }
        catch (java.io.IOException exc) {
            if (SAVE_IMAGE_FILE_LOG) BetaLog.dd(SAVE_IMAGE_FILE_TAG, "IOException on File Close");
            return false;
        }

        return true;

    }

    /**
     * Saves RGB565 image to a file using default file name, with or without a datestamp appended to name.
     * @param imageBytes Byte array containing the RGB565 image, 2 bytes per pixel, little-endian order
     * @param imgWidth Image width
     * @param imgHeight Image height
     * @param dateStamp true if a date stamp is wanted in file name, otherwise false
     * @return true if successful, otherwise false
     */
    public static boolean saveImageFile(byte[] imageBytes, int imgWidth, int imgHeight, boolean dateStamp){

        GregorianCalendar gregorianCalendar = new GregorianCalendar();

        String path;
        if (dateStamp) path = String.format("/sdcard/DCIM/ImageBytes%04d%02d%02d%02d%02d%02d.dat",
                gregorianCalendar.get(Calendar.YEAR), gregorianCalendar.get(Calendar.MONTH),
                gregorianCalendar.get(Calendar.DAY_OF_MONTH), gregorianCalendar.get(Calendar.HOUR_OF_DAY),
                gregorianCalendar.get(Calendar.MINUTE), gregorianCalendar.get(Calendar.SECOND));
        else path = "/sdcard/DCIM/ImageBytes.dat";

        return saveImageFile(imageBytes, imgWidth, imgHeight, path);

    }


}
