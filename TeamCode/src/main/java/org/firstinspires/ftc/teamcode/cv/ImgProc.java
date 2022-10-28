package org.firstinspires.ftc.teamcode.cv;

import android.content.Context;

import org.firstinspires.ftc.teamcode.cv.HSV_Range;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * Provides static image processing methods.
 */

public class ImgProc {

    /**
     * Perform separable convolution of source image src with array g. g is used for convolution in
     * both the x and y directions. The number of elements in g is assumed to be ODD.
     */
//    public static void separableConvolution(int width, int height, float[] src, float[] g, float[] dst) {
//        float[] tmp = new float[width * height];
//        int nG = g.length;
//        int nG2 = nG / 2;
//
//        //Start with the horizontal convolution
//        for (int y = 0; y < height; y++) {
//            int yIndex = y * width;
//            for (int x = 0; x < width; x++) {
//                int index = yIndex + x;
//                float sum = 0.0f;
//                for (int n = Math.max(0, nG2 + x + 1 - width); n < Math.min(nG, nG2 + x + 1); n++) {
//                    sum += g[n] * src[index + nG2 - n];
//                }
//                tmp[index] = sum;
//            }
//        }
//
//        //Now do the vertical convolution
//        for (int y = 0; y < height; y++) {
//            int yIndex = y * width;
//            for (int x = 0; x < width; x++) {
//                int index = yIndex + x;
//                float sum = 0.0f;
//                for (int n = Math.max(0, nG2 + y + 1 - height); n < Math.min(nG, nG2 + y + 1); n++) {
//                    sum += g[n] * tmp[index + (nG2 - n) * width];
//                }
//                dst[index] = sum;
//            }
//        }
//
//    }

    /*
    getReducedRGB565
    Samples an array (src) of bytes representing RGB565 image and fills an array (dst) representing
    a reduced image.
     */
//    public static void getReducedRGB565(byte[] src, int srcWidth, int srcHeight, byte[] dst, int dstWidth, int dstHeight) {
//        int srcRowIndex, dstRowIndex;
//        int widthRatio = srcWidth / dstWidth;
//        int heightRatio = srcHeight / dstHeight;
//        for (int dstRow = 0; dstRow < dstHeight; dstRow++) {
//            dstRowIndex = 2 * dstRow * dstWidth;
//            srcRowIndex = 2 * dstRow * heightRatio * srcWidth;
//            for (int dstCol = 0; dstCol < dstWidth; dstCol++) {
//                int dstIndex = dstRowIndex + 2 * dstCol;
//                int srcIndex = srcRowIndex + 2 * dstCol * widthRatio;
//                dst[dstIndex] = src[srcIndex];
//                dst[dstIndex + 1] = src[srcIndex + 1];
//            }
//        }
//    }

    /*
    getRangeRGB565
    Obtains a rectangular range of pixels from an array (src) of bytes representing an RGB565 image and places them
    in an array of bytes (dst) which also represents an RGB565 image.
     */
//    public static void getRangeRGB565(byte[] src, int srcWidth, int srcHeight, int x0, int y0, byte[] dst, int dstWidth, int dstHeight){
//        int srcRowIndex, dstRowIndex, dstIndex, srcIndex;
//        for (int dstRow = 0; dstRow < dstHeight; dstRow++){
//            dstRowIndex = 2 * dstRow * dstWidth;
//            srcRowIndex = 2 * ((y0 + dstRow) * srcWidth);
//            for (int dstCol = 0; dstCol < dstWidth; dstCol++){
//                dstIndex = dstRowIndex + 2 * dstCol;
//                srcIndex = srcRowIndex + 2 * (x0 + dstCol);
//                dst[dstIndex] = src[srcIndex];
//                dst[dstIndex+1] = src[srcIndex+1];
//            }
//        }
//    }

    /**
     * Reduces the size of an RGB565 image by cropping (i.e., selecting a subRange of the original
     * image), then sampling the subrange at fixed row and column intervals.
     *
     * @param src Byte array containing the source RGB565 pixels, two bytes per pixels, little-endian order.
     * @param srcWidth Width of the source image.
     * @param srcHeight Height of the source image.
     * @param x0 Index of the first source image column to include in the sub-range.
     * @param y0 Index of the first source image row to include in in the sub-range.
     * @param rangeWidth Width of the sub-range.
     * @param rangeHeight Height of the sub-range.
     * @param dst Byte array to receive the reduced, sampled image
     * @param sampleRatio Period at which to sample the sub-range columns and rows (every 1st, every 2nd, etc)
     */

    public static void getReducedRangeRGB565(byte[] src, int srcWidth, int srcHeight, int x0, int y0,
                                             int rangeWidth, int rangeHeight, byte[] dst, int sampleRatio)
    {
        if (src.length < (2 * srcWidth * srcHeight)) throw new IllegalArgumentException("Source array too small for width and height");
        if (x0 < 0 || y0 < 0 || x0+rangeWidth > srcWidth || y0+rangeHeight > srcHeight)
            throw new IllegalArgumentException("Invalid subrange");
        if (rangeWidth % sampleRatio != 0 || rangeHeight % sampleRatio != 0)
            throw new IllegalArgumentException("sampleRatio not a common divisor of rangeWidth and rangeHeight");
        int widthRatio = sampleRatio;
        int heightRatio = sampleRatio;
        int dstWidth = rangeWidth / sampleRatio;
        int dstHeight = rangeHeight / sampleRatio;

        for (int y = 0; y < dstHeight; y++)
        {
            int srcRowIndex = 2 * srcWidth * (y0 + heightRatio * y);
            for (int x = 0; x < dstWidth; x++)
            {
                int srcIndex = srcRowIndex + 2 * (x0 + widthRatio * x);
                int dstIndex = 2 * (y * dstWidth + x);
                dst[dstIndex] = src[srcIndex];
                dst[dstIndex + 1] = src[srcIndex + 1];
            }
        }
    }




    /**
     * Return normalized (sum = 1) gaussian array with length of 2*halfWidth+1, peak at
     * index halfWidth, and standard deviation of sigma
     */
//    public static float[] getGaussian(float sigma, int halfWidth) {
//        int nG = 2 * halfWidth + 1;
//        float[] result = new float[nG];
//        for (int n = 0; n < nG; n++) {
//            result[n] = (float) Math.exp(-(n - halfWidth) * (n - halfWidth) / (2.0 * sigma));
//        }
//        float sum = 0;
//        for (int n = 0; n < nG; n++) sum += result[n];
//        for (int n = 0; n < nG; n++) result[n] /= sum;
//        return result;
//    }

    /**
     * Compares each pixel of an RGB565 image with the accepted range of hue, saturation, and value,
     * and creates a new "Binary Image"--an array of integers with values of 0 (no match) or 1 (match).
     *
     * Note: for target colors near red, which has hue of 0, the values used for minHue and maxHue
     * may not be intuitive. For example, to bracket a hue of 0 +/- 30, you would use minHue of 330
     * and maxHue of 30.
     *
     * @param src Byte array containing the RGB565 pixels of the source image.
     * @param minHue Minimum hue to be accepted as a match.
     * @param maxHue Maximum hue to be accepted as a match.
     * @param minSat Minimum saturation to be accepted as a match.
     * @param maxSat Maximum saturation to be accepted as a match.
     * @param minVal Minimum value to be accepted as a match.
     * @param maxVal Maximum value to be accepted as a match.
     * @param dst Int array to receive the binary image.
     */


    public static void getBinaryImage(byte[] src, float minHue, float maxHue, float minSat,
                                      float maxSat, float minVal, float maxVal, int[] dst) {
        int red, green, blue;
        int max, min, c; //These will ultimately hold max(r,g,b), min(r,g,b) and chroma, which is max-min
        int numPixels = src.length / 2; //RGB565 stores each pixel as two bytes
        int i1, i2;
        float hPrime;
        //Use min and max hue in the 0-6 range to avoid repeated multiplication by 60f
        float minHue06 = minHue / 60.0f;
        float maxHue06 = maxHue / 60.0f;
        //Use integer min and max value in the 0-255 range to avoid repeated division by 255f
        int minVal255 = Math.round(minVal * 255.0f);
        int maxVal255 = Math.round(maxVal * 255.0f);
        float sat;

        for (int i = 0; i < numPixels; i++) {
            dst[i] = 0; //Set the binary destination pixel to 0; we'll change it to 1 if it proves a match
            i1 = 2 * i;
            i2 = 2 * i + 1;

            //Extract the red, green, and blue values from the RGB565 bytes for this pixel
            blue = (src[i1] & 0x1F) << 3;
            red = (src[i2] & 0xF8);
            green = ((src[i1] & 0xE0) >> 3) + ((src[i2] & 0x7) << 5);

            //Do in-line conversion from RGB565 to HSV to avoid repeated function calls
            //Note: Rather than computing hue in the 0-360 range, will use hPrime in the 0 to 6 range
            //Actual hue would be obtained by multiplying by 60, but that would be wasteful

            if (red > green) {
                if (red > blue) {
                    //Red is Maximum, either Green or Blue may be Minimum
                    max = red;
                    min = Math.min(green, blue);
                    c = max - min;
                    hPrime = c == 0 ? 0 : (float) (green - blue) / (float) c + 6.0f;
                    hPrime = hPrime >= 6.0f ? hPrime - 6.0f : hPrime;
                } else {
                    //Blue is Maximum, Green is Minimum
                    max = blue;
                    c = max - green;
                    hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
                }
            } else {
                if (green > blue) {
                    //Green is Maximum, either Red or Blue may be Minimum
                    max = green;
                    min = Math.min(red, blue);
                    c = max - min;
                    hPrime = c == 0? 0 : (float) (blue - red) / (float) c + 2.0f;
                } else {
                    //Blue is Maximum, Red is Minimum
                    max = blue;
                    c = max - red;
                    hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
                }
            }

            //Check if hue is a match. If not, move on to the next pixel
            if ( (maxHue >= minHue && (hPrime < minHue06 || hPrime > maxHue06))
                    || (maxHue < minHue && hPrime > maxHue06 && hPrime < minHue06) ) continue;

            //Saturation is just the chroma divided by the max value among r,g,b (unless the max value is 0)
            sat = max > 0 ? (float) c / (float) max : 0;

            //Check if saturation and value are a match. If not, move on to the next pixel
            if (sat < minSat || sat > maxSat || max < minVal255 || max > maxVal255) continue;

            //To reach this point, hue, saturation, and value must all match. Set binary pixel to 1.
            dst[i] = 1;
        }
    }

    /**
     * Convenience getBinaryImage method that accepts an HSV_Range object as a parameter
     * @param src Byte array containing the RGB565 source image pixels.
     * @param hsvRange HSV_Range object containing the target range of hsv values.
     * @param dst Int array to hold the binary image.
     */
    public static void getBinaryImage(byte[] src, HSV_Range hsvRange, int[] dst){
        getBinaryImage(src, hsvRange.getMinHue(), hsvRange.getMaxHue(), hsvRange.getMinSat(), hsvRange.getMaxSat(),
                hsvRange.getMinVal(), hsvRange.getMaxVal(), dst);
    }



//    public static void reduceBinaryImageAvg(int[] src, int srcWidth, int srcHeight, float[] dst, int dstWidth, int dstHeight) {
//        int srcRowIndex, dstRowIndex;
//        int widthRatio = srcWidth / dstWidth;
//        int heightRatio = srcHeight / dstHeight;
//        int srcPixelsPerBlock = widthRatio * heightRatio;
//        for (int dstRow = 0; dstRow < dstHeight; dstRow++) {
//            dstRowIndex = dstRow * dstWidth;
//            srcRowIndex = dstRow * heightRatio * srcWidth;
//            for (int dstCol = 0; dstCol < dstWidth; dstCol++) {
//                int dstIndex = dstRowIndex + dstCol;
//                int srcIndex = srcRowIndex + widthRatio * dstCol;
//                int tmp = 0;
//                for (int y = 0; y < heightRatio; y++)
//                    for (int x = 0; x < widthRatio; x++)
//                        if (src[srcIndex + y * srcWidth + x] == 1) tmp += 1;
//                dst[dstIndex] = tmp / srcPixelsPerBlock;
//            }
//        }
//    }


    //Get red, green, blue from the specified pixel in an RGB565 byte array
//    public static int[] getRGBfromByteArray(int row, int col, int width, byte[] src){
//        int index = 2 * (row * width + col);
//        byte b1 = src[index];
//        byte b2 = src[index+1];
//        int blue = (b1 & 0x1F) << 3;
//        int red = (b2 & 0xF8);
//        int green = ((b1 & 0xE0) >> 3) + ((b2 & 0x7) << 5);
//        return new int[] {red, green, blue};
//    }

    //Get hsv from a color integer
//    public static void colorToHSV(int col, float[] hsv) {
//        int c;
//        int max;
//        int min;
//        int blue = col & 0xFF;
//        int green = (col >> 8) & 0xFF;
//        int red = (col >> 16) & 0xFF;
//        float hPrime;
//
//        if (red > green) {
//            if (red > blue) {
//                //Red is Maximum, either Green or Blue may be Minimum
//                max = red;
//                min = Math.min(green, blue);
//                c = max - min;
//                hPrime = c == 0 ? 0 : (float) (green - blue) / (float) c + 6.0f;
//                hPrime = hPrime >= 6.0f ? hPrime - 6.0f : hPrime;
//            } else {
//                //Blue is Maximum, Green is Minimum
//                max = blue;
//                c = max - green;
//                hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
//            }
//        } else {
//            if (green > blue) {
//                //Green is Maximum, either Red or Blue may be Minimum
//                max = green;
//                min = Math.min(red, blue);
//                c = max - min;
//                hPrime = c == 0? 0 : (float) (blue - red) / (float) c + 2.0f;
//            } else {
//                //Blue is Maximum, Red is Minimum
//                max = blue;
//                c = max - red;
//                hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
//            }
//        }
//
//        hsv[0] = hPrime * 60.0f;
//        hsv[1] = max > 0 ? (float) c / (float) max : 0;
//        hsv[2] = (float) max / 255.0f;
//
//    }
//
//    public static void rgb565ToHSV(byte b1, byte b2, float[] hsv) {
//        int c;
//        int max;
//        int min;
//        int blue = (b1 & 0x1F) << 3;
//        int red = (b2 & 0xF8);
//        int green = ((b1 & 0xE0) >> 3) + ((b2 & 0x7) << 5);
//        float hPrime;
//
//        if (red > green) {
//            if (red > blue) {
//                //Red is Maximum, either Green or Blue may be Minimum
//                max = red;
//                min = Math.min(green, blue);
//                c = max - min;
//                hPrime = c == 0 ? 0 : (float) (green - blue) / (float) c + 6.0f;
//                hPrime = hPrime >= 6.0f ? hPrime - 6.0f : hPrime;
//            } else {
//                //Blue is Maximum, Green is Minimum
//                max = blue;
//                c = max - green;
//                hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
//            }
//        } else {
//            if (green > blue) {
//                //Green is Maximum, either Red or Blue may be Minimum
//                max = green;
//                min = Math.min(red, blue);
//                c = max - min;
//                hPrime = c == 0? 0 : (float) (blue - red) / (float) c + 2.0f;
//            } else {
//                //Blue is Maximum, Red is Minimum
//                max = blue;
//                c = max - red;
//                hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
//            }
//        }
//
//        hsv[0] = hPrime * 60.0f;
//        hsv[1] = max > 0 ? (float) c / (float) max : 0;
//        hsv[2] = (float) max / 255.0f;
//
//    }



}
