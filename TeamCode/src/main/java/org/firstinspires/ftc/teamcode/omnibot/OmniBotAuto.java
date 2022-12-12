package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Predicate;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.KalmanMeasurementUpdater;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.List;

public abstract class OmniBotAuto extends LinearOpMode {

    public static float YMAX = 141;

    protected OmniBot bot = new OmniBot();

    public enum SignalResult{ONE, TWO, THREE}
    protected SignalResult signalResult = SignalResult.ONE;

    public enum Quadrant{ RED_RIGHT, RED_LEFT, BLUE_RIGHT, BLUE_LEFT }

    public enum HeadingIndex{
        H_0 (0),
        H_90 ((float)Math.PI/2),
        H_NEG_90(-(float)Math.PI/2),
        H_180 ((float)Math.PI);

        public float radians;
        HeadingIndex(float radians){ this.radians = radians; }
    }

    HSV_Range hsvGreen = new HSV_Range(80, 120, 0.25f, 1.0f, 0.3f, 1.0f);

    public interface Pred{
        boolean test();
    }

    /*
    VUFORIA TARGET POSITIONS
     */
    protected static final float TARGET_X = 34.75f * 25.4f; //mm
    protected static final float LEFT_TARGET_Y = 141.0f * 25.4f; //mm  (Note that right target Y is 0)
    protected static final OpenGLMatrix RIGHT_TARGET_LOCATION = OpenGLMatrix.translation(TARGET_X, 0,0)
            .multiplied(Orientation.getRotationMatrix(AxesReference.INTRINSIC, AxesOrder.ZXZ,
                    AngleUnit.DEGREES, 180, 90, 0));
    protected static final OpenGLMatrix LEFT_TARGET_LOCATION = OpenGLMatrix.translation(TARGET_X, LEFT_TARGET_Y, 0)
            .multiplied(Orientation.getRotationMatrix(AxesReference.INTRINSIC, AxesOrder.XYZ,
                    AngleUnit.DEGREES, 90, 0, 0));
    public static final OpenGLMatrix[] TARGET_LOCATIONS = new OpenGLMatrix[]{
            LEFT_TARGET_LOCATION, // Red Left, aka Red Audience side
            RIGHT_TARGET_LOCATION, // Red Right, aka Red Back wall
            RIGHT_TARGET_LOCATION, // Blue Right, aka Blue Audience side
            LEFT_TARGET_LOCATION // Blue Left, aka Blue Back wall
    };

    protected SignalResult getSignalResult(){
        BlobHelper blobHelper = new BlobHelper(640, 480, 120, 0,
                400, 320, 1);
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



    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees,
                                   float cp, float tolerance, KalmanMeasurementUpdater updater) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

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

    public void driveDirection(float speed, float directionDegrees, float targetHeading, float cp,
                          Pred finished){
        float directionRadians = (float)Math.toRadians(directionDegrees);
        float targetHeadingRadians = (float)Math.toRadians(targetHeading);
        while(opModeIsActive()){
            bot.updateOdometry();
            if(finished.test()){
                break;
            }
            float thetaError = (float)AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);
            float va = 1.0f * thetaError;
            float vx = speed * (float)Math.cos(directionRadians);
            float vy = speed * (float)Math.sin(directionRadians);
            float vxr = vx * sinTheta - vy * cosTheta;
            float vyr = vx * cosTheta + vy * sinTheta;
            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }

    public void driveDirection(float speed, float directionDegrees, float targetHeading, float cp,
                               KalmanMeasurementUpdater updater, Pred finished){
        float directionRadians = (float)Math.toRadians(directionDegrees);
        float targetHeadingRadians = (float)Math.toRadians(targetHeading);
        while(opModeIsActive()){
            bot.updateOdometry();
            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }
            if(finished.test()){
                break;
            }
            float thetaError = (float)AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);
            float va = 1.0f * thetaError;
            float vx = speed * (float)Math.cos(directionRadians);
            float vy = speed * (float)Math.sin(directionRadians);
            float vxr = vx * sinTheta - vy * cosTheta;
            float vyr = vx * cosTheta + vy * sinTheta;
            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }

    /**
     *
     * @param quad      Quadrant of field
     * @param index     Bot heading index
     * @param p0        Starting point on line (needn't be exact bot starting position)
     * @param directionDegrees  Direction of travel along line
     * @param profile   Motion profile (vMax, vMin, accel)
     * @param cpTheta   Correction coefficient for heading (orientation)
     * @param cpDist    Correction coefficient for linear offset from line
     * @param estDist   Estimated total distance of travel along the line
     * @param updater   KalmanMeasurementUpdater for pose correction (may be null if desired)
     * @param finished  Pred object to indicate when operation is finished
     */
    public void driveLine(Quadrant quad, HeadingIndex index, VectorF p0, float directionDegrees,
                               MotionProfile profile, float cpTheta, float cpDist, float estDist,
                               KalmanMeasurementUpdater updater, Pred finished){
        float targetHeadingRadians = index.radians;
        float directionRadians = (float)Math.toRadians(directionDegrees);

        // Unit vector indicating direction of travel along line
        VectorF u = new VectorF((float)Math.cos(directionRadians), (float)Math.sin(directionRadians));

        // Unit vector perpendicular to direction of travel along line (w = z x u)
        VectorF w = new VectorF(-u.get(1), u.get(0));

        float vMinSqr = profile.vMin * profile.vMin;

        while (opModeIsActive()){

            // Update pose, using odometry followed by Kalman Updater (if provided)
            bot.updateOdometry();
            if (updater != null){
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

            // Check to see if it is time to terminate the operation
            if (finished.test()) break;

            // Vector from starting position of line to current bot position
            VectorF q = new VectorF(bot.getPose().x, bot.getPose().y).subtracted(p0);

            // Vector from current robot position to closest point on line
            VectorF err = w.multiplied(q.get(0)*u.get(1) - q.get(1)*u.get(0));

            // Distance from p0 to point on line closest to robot (q * u); don't allow neg value
            float d0 = Math.max(0, q.dotProduct(u));
            // Estimated distance from bot position to stopping point; don't allow neg value
            float d1 = Math.max(0, estDist - d0);

            /*
             * Target speed will be the minimum of: vMax and the speeds calculated from the
             * acceleration/deceleration, vMin, and distance from start or end point.
             */
            float speed = (float)Math.min(profile.vMax,
                    Math.min(Math.sqrt(vMinSqr + 2*profile.accel*d0),
                            Math.sqrt(vMinSqr + 2*profile.accel*d1)));

            /*
             * Get velocity by adding a correction to the nominal velocity along the line.
             */
            VectorF vel = u.multiplied(speed).added(err.multiplied(cpDist));

            // Convert to velocity in robot coordinate system
            float sin = (float)Math.sin(bot.getPose().theta);
            float cos = (float)Math.cos(bot.getPose().theta);
            float vxr = vel.get(0) * sin - vel.get(1) * cos;
            float vyr = vel.get(0) * cos + vel.get(1) * sin;

            // Angular speed based on offset from target heading
            float headingOffset = AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float va = cpTheta * headingOffset;

            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0,0,0);
        bot.updateOdometry();
    }

    public void adjustPositionColor(float targetHue, float targetHeadingDegrees, float maxSpeed,
                                    float cpHue, float cpHeading, float tolerance){
        float targetHeadingRadians = (float)Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive()){
            bot.updateOdometry();
            float hueOffset = targetHue - bot.getHSV()[1];

            if (Math.abs(hueOffset) < tolerance){
                break;
            }

            float vx = maxSpeed * cpHue * hueOffset;
            float vxr = vx * (float)Math.sin(bot.getPose().theta);
            float vyr = vx * (float)Math.cos(bot.getPose().theta);
            float va = cpHeading * (float)AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);

            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0,0,0);
        bot.updateOdometry();
    }


    public class PowerPlayDistUpdater extends KalmanDistanceUpdater {

        public PowerPlayDistUpdater(Quadrant quadrant, HeadingIndex index, boolean measX,
                                    boolean measY, Predicate<Float> xValid, Predicate<Float> yValid){
            this(quadrant, index, measX, measY);
            this.xValid = xValid;
            this.yValid = yValid;
        }

        public PowerPlayDistUpdater(Quadrant quadrant, HeadingIndex index, boolean measX, boolean measY){

            xFromDist = (d) -> d + 6;
            xValid = (d) -> d>3 && d<60;
            yValid = (d) -> d>3 && d<60;

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
