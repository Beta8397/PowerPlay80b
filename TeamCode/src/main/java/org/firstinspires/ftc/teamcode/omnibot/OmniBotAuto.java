package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;

import java.util.List;

public abstract class OmniBotAuto extends LinearOpMode {

    public static float YMAX = 141;

    protected OmniBot bot = new OmniBot();

    public enum SignalResult{ONE, TWO, THREE}
    protected SignalResult signalResult = SignalResult.ONE;

    public enum Quadrant{ RED_RIGHT, RED_LEFT, BLUE_RIGHT, BLUE_LEFT }
    public enum HeadingIndex{ H_0, H_90, H_NEG_90, H_180 }

    HSV_Range hsvGreen = new HSV_Range(90, 150, 0.3f, 1.0f, 0.3f, 1.0f);

    protected SignalResult getSignalResult(){
        BlobHelper blobHelper = new BlobHelper(640, 480, 0, 0,
                640, 480, 2);
        while(opModeIsActive() && !blobHelper.updateImage()) continue;
        List<Blob> blobs = blobHelper.getBlobs(hsvGreen);
        if(blobs.size() == 0) return SignalResult.ONE;
        while(blobs.size() > 1){
            if(blobs.get(0).getNumPts() > blobs.get(1).getNumPts()){
                blobs.remove(1);
            } else{
                blobs.remove(0);
            }
        }
        Blob biggestBlob = blobs.get(0);
        float angle = biggestBlob.getAngle();
        if(angle > 0 && angle < Math.PI/3) return SignalResult.ONE;
        if(angle < 0 && angle > -Math.PI/3) return SignalResult.THREE;
        return SignalResult.TWO;
    }

    /**
     * Turn to the specified heading using proportionate control
     *
     * @param targetHeadingDegrees
     * @param toleranceDegrees
     * @param propCoeff
     */
    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float maxDegreesPerSec) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        while (opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtil.normalizeRadians(targetHeadingRadians - currentHeading);
            if (Math.abs(angleDiff) < toleranceRadians) break;

            float va = propCoeff * angleDiff;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }


    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees,
                                   float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtil.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = 1.0f * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    public class PowerPlayDistUpdater extends KalmanDistanceUpdater {

        public PowerPlayDistUpdater(Quadrant quadrant, HeadingIndex index, boolean measX,
                                    boolean measY, Predicate xValid, Predicate yValid){
            this(quadrant, index, measX, measY);
            this.xValid = xValid;
            this.yValid = yValid;
        }

        private PowerPlayDistUpdater(Quadrant quadrant, HeadingIndex index, boolean measX, boolean measY){

            xFromDist = (d) -> d + 6;
            xValid = (d) -> d>3 && d<48;
            yValid = (d) -> d>3 && d<48;

            if (quadrant == Quadrant.RED_RIGHT || quadrant == Quadrant.BLUE_RIGHT){
                yFromDist = (d) -> d + 6;
                switch (index) {
                    case H_0:
                        if (measX) sensX = bot.backDist;
                        if (measY) sensY = bot.rightDist;
                        break;
                    case H_90:
                        if (measX) sensX = bot.leftDist;
                        if (measY) sensY = bot.backDist;
                        break;
                    case H_NEG_90:
                        if (measX) sensX = bot.rightDist;
                        if (measY) sensY = bot.frontDist;
                        break;
                    case H_180:
                        if (measX) sensX = bot.frontDist;
                        if (measY) sensY = bot.leftDist;
                        break;
                }
            } else {
                yFromDist = (d) -> YMAX - d - 6;
                switch (index) {
                    case H_0:
                        if (measX) sensX = bot.backDist;
                        if (measY) sensY = bot.leftDist;
                        break;
                    case H_90:
                        if (measX) sensX = bot.leftDist;
                        if (measY) sensY = bot.frontDist;
                        break;
                    case H_NEG_90:
                        if (measX) sensX = bot.rightDist;
                        if (measY) sensY = bot.backDist;
                        break;
                    case H_180:
                        if (measX) sensX = bot.frontDist;
                        if (measY) sensY = bot.rightDist;
                        break;
                }
            }
        }
    }

}
