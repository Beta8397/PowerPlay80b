package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Predicate;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
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
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

import java.util.List;

public abstract class OmniBotAuto extends LinearOpMode {

    public static float YMAX = 141;
    public static float X_TAPE_EDGE = 57.5f;

    protected OmniBot bot = new OmniBot();

    public final MotionProfile midSpeed = new MotionProfile(10, 16, 16);
    public final MotionProfile highSpeed = new MotionProfile(10, 20, 16);
    public final MotionProfile ultraHighSpeed = new MotionProfile(16, 32, 24);
    public final MotionProfile lowSpeed = new MotionProfile(6, 6, 0);
    public final MotionProfile safeSpeed = new MotionProfile(8, 14, 14);

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
            float va = 2.0f * thetaError;
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
            float va = 2.0f * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile profile, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance, KalmanMeasurementUpdater updater) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        //Starting point
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        while (opModeIsActive()) {

            // Update bot pose with odometry, and if provided, adjust with Kalman filter
            bot.updateOdometry();

            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

            // Distance of bot from starting point
            float d0 = (float)Math.hypot(bot.getPose().x - x0, bot.getPose().y - y0);

            // Vector (field coordinates) from bot to target point; d1 is magnitude of this vector
            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);

            if (d1 < tolerance) break;

            // Heading error
            float thetaError = (float)AngleUtil.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            // Convert the (xError, yError) vector to ROBOT coordinate system (xErrorRobot, yErrorRobot)
            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);
            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            /*
             * Robot speed will be the minimum of vMax and the two speeds computed from vMin,
             * acceleration, and distances from starting and target points
             */
            float vMinSqr = profile.vMin * profile.vMin;
            float speed = (float)Math.min(profile.vMax,
                    Math.min(Math.sqrt(vMinSqr + 2*d0*profile.accel),
                            Math.sqrt(vMinSqr + 2*d1*profile.accel)));

            // Robot velocity vector
            float vxr = speed * xErrorRobot / d1;
            float vyr = speed * yErrorRobot / d1;
            float va = 2.0f * thetaError;
            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile profile, float targetX, float targetY, float targetThetaDegrees,
                                         float tolerance, WiggleProfile wiggleProfile, KalmanMeasurementUpdater updater) {
        float headingTargetRadiansBase = targetThetaDegrees * (float) Math.PI / 180;

        //Starting point
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        ElapsedTime et = new ElapsedTime();


        while (opModeIsActive()) {

            // Update bot pose with odometry, and if provided, adjust with Kalman filter
            bot.updateOdometry();

            float headingTargetRadians;
            float seconds = (float) et.seconds();

            if (seconds < wiggleProfile.seconds) {
                headingTargetRadians = headingTargetRadiansBase +
                        wiggleProfile.amplitude *
                                (float) Math.sin(2 * Math.PI * wiggleProfile.frequency * seconds);
            } else {
                headingTargetRadians = headingTargetRadiansBase;
            }

            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

            // Distance of botfrom starting point
            float d0 = (float) Math.hypot(bot.getPose().x - x0, bot.getPose().y - y0);

            // Vector (field coordinates) from bot to target point; d1 is magnitude of this vector
            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float) Math.hypot(xError, yError);

            if (d1 < tolerance) break;

            // Heading error
            float thetaError = (float) AngleUtil.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            // Convert the (xError, yError) vector to ROBOT coordinate system (xErrorRobot, yErrorRobot)
            float sinTheta = (float) Math.sin(bot.getPose().theta);
            float cosTheta = (float) Math.cos(bot.getPose().theta);
            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            /*
             * Robot speed will be the minimum of vMax and the two speeds computed from vMin,
             * acceleration, and distances from starting and target points
             */
            float vMinSqr = profile.vMin * profile.vMin;
            float speed = (float) Math.min(profile.vMax,
                    Math.min(Math.sqrt(vMinSqr + 2 * d0 * profile.accel),
                            Math.sqrt(vMinSqr + 2 * d1 * profile.accel)));

            // Robot velocity vector
            float vxr = speed * xErrorRobot / d1;
            float vyr = speed * yErrorRobot / d1;
            float va = 2.0f * thetaError;
            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }




    protected void driveToPosition(MotionProfile profile, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance, WiggleProfile wiggleProfile, Runnable runnable,
                                   KalmanMeasurementUpdater updater) {
        float headingTargetRadiansBase = targetThetaDegrees * (float) Math.PI / 180;

        //Starting point
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        ElapsedTime et = new ElapsedTime();


        while (opModeIsActive()) {

            // Update bot pose with odometry, and if provided, adjust with Kalman filter
            bot.updateOdometry();

            float headingTargetRadians;
            float seconds = (float) et.seconds();

            if (seconds < wiggleProfile.seconds) {
                headingTargetRadians = headingTargetRadiansBase +
                        wiggleProfile.amplitude *
                                (float) Math.sin(2 * Math.PI * wiggleProfile.frequency * seconds);
            } else {
                headingTargetRadians = headingTargetRadiansBase;
            }

            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

            // Distance of botfrom starting point
            float d0 = (float) Math.hypot(bot.getPose().x - x0, bot.getPose().y - y0);

            // Vector (field coordinates) from bot to target point; d1 is magnitude of this vector
            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float) Math.hypot(xError, yError);

            if (d1 < tolerance) break;

            if (runnable != null) {
                runnable.run();
            }

            // Heading error
            float thetaError = (float) AngleUtil.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            // Convert the (xError, yError) vector to ROBOT coordinate system (xErrorRobot, yErrorRobot)
            float sinTheta = (float) Math.sin(bot.getPose().theta);
            float cosTheta = (float) Math.cos(bot.getPose().theta);
            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            /*
             * Robot speed will be the minimum of vMax and the two speeds computed from vMin,
             * acceleration, and distances from starting and target points
             */
            float vMinSqr = profile.vMin * profile.vMin;
            float speed = (float) Math.min(profile.vMax,
                    Math.min(Math.sqrt(vMinSqr + 2 * d0 * profile.accel),
                            Math.sqrt(vMinSqr + 2 * d1 * profile.accel)));

            // Robot velocity vector
            float vxr = speed * xErrorRobot / d1;
            float vyr = speed * yErrorRobot / d1;
            float va = 2.0f * thetaError;
            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0, 0, 0);
        bot.updateOdometry();
    }

    protected void driveToPositionWiggle(MotionProfile profile, float targetX, float targetY, float targetThetaDegrees,
                                         float tolerance, float wiggleTime, float wiggleAmplitude,
                                         float wiggleFrequency, KalmanMeasurementUpdater updater) {
        float headingTargetRadiansBase = targetThetaDegrees * (float)Math.PI / 180;

        //Starting point
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        ElapsedTime et = new ElapsedTime();


        while (opModeIsActive()) {

            // Update bot pose with odometry, and if provided, adjust with Kalman filter
            bot.updateOdometry();

            float headingTargetRadians;
            float seconds =  (float)et.seconds();

            if(seconds < wiggleTime){
                headingTargetRadians = headingTargetRadiansBase +
                        wiggleAmplitude *
                                (float)Math.sin(2 * Math.PI * wiggleFrequency * seconds);
            } else{
                headingTargetRadians = headingTargetRadiansBase;
            }

            if (updater != null) {
                Pose updatedPose = updater.updatePose(bot.getPose(), bot.getCovariance());
                bot.adjustPose(updatedPose.x, updatedPose.y);
            }

            // Distance of botfrom starting point
            float d0 = (float)Math.hypot(bot.getPose().x - x0, bot.getPose().y - y0);

            // Vector (field coordinates) from bot to target point; d1 is magnitude of this vector
            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);

            if (d1 < tolerance) break;

            // Heading error
            float thetaError = (float)AngleUtil.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            // Convert the (xError, yError) vector to ROBOT coordinate system (xErrorRobot, yErrorRobot)
            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);
            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            /*
             * Robot speed will be the minimum of vMax and the two speeds computed from vMin,
             * acceleration, and distances from starting and target points
             */
            float vMinSqr = profile.vMin * profile.vMin;
            float speed = (float)Math.min(profile.vMax,
                    Math.min(Math.sqrt(vMinSqr + 2*d0*profile.accel),
                            Math.sqrt(vMinSqr + 2*d1*profile.accel)));

            // Robot velocity vector
            float vxr = speed * xErrorRobot / d1;
            float vyr = speed * yErrorRobot / d1;
            float va = 2.0f * thetaError;
            bot.setDriveSpeed(vxr, vyr, va);
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
     * @param targetHeadingDegrees     desired orientation of robot
     * @param p0        Starting point on line (needn't be exact bot starting position)
     * @param directionDegrees  Direction of travel along line
     * @param profile   Motion profile (vMax, vMin, accel)
     * @param cpDist    Correction coefficient for linear offset from line
     * @param estDist   Estimated total distance of travel along the line
     * @param updater   KalmanMeasurementUpdater for pose correction (may be null if desired)
     * @param finished  Pred object to indicate when operation is finished
     */
    public void driveLine(float targetHeadingDegrees, VectorF p0, float directionDegrees,
                               MotionProfile profile, float cpDist, float estDist,
                               KalmanMeasurementUpdater updater, Pred finished){
        float targetHeadingRadians = (float)Math.toRadians(targetHeadingDegrees);
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
            float va = 2.0f * headingOffset;

            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0,0,0);
        bot.updateOdometry();
    }

    public void adjustPositionColor(float targetSat, float targetHeadingDegrees, float maxSpeed,
                                    float cpSat, float tolerance){
        float targetHeadingRadians = (float)Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive()){
            bot.updateOdometry();
            float satOffset = targetSat - bot.getHSV()[1];

            if (Math.abs(satOffset) < tolerance){
                break;
            }

            float vx = maxSpeed * cpSat * satOffset;
            float vxr = vx * (float)Math.sin(bot.getPose().theta);
            float vyr = vx * (float)Math.cos(bot.getPose().theta);
            float va = 2.0f * (float)AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);

            bot.setDriveSpeed(vxr, vyr, va);
        }
        bot.setDriveSpeed(0,0,0);
        bot.updateOdometry();
    }

    public void adjustPositionColor(float targetSat, float xOffset, float targetHeadingDegrees, float maxSpeed,
                                    float cpSat, float tolerance){
        float targetHeadingRadians = (float)Math.toRadians(targetHeadingDegrees);
        while (opModeIsActive()){
            bot.updateOdometry();
            float satOffset = targetSat - bot.getHSV()[1];

            if (Math.abs(satOffset) < tolerance){
                break;
            }

            float vx = maxSpeed * cpSat * satOffset;
            float vxr = vx * (float)Math.sin(bot.getPose().theta);
            float vyr = vx * (float)Math.cos(bot.getPose().theta);
            float va = 2.0f * (float)AngleUtil.normalizeRadians(targetHeadingRadians - bot.getPose().theta);

            bot.setDriveSpeed(vxr, vyr, va);
        }
        driveToPosition(maxSpeed, maxSpeed/2, bot.getPose().x + xOffset, bot.getPose().y, targetHeadingDegrees, 2, 0.5f);
        bot.updateOdometry();
    }

    public void driveTapeToStack45(Quadrant quadrant, float xOffset, float initHeadingDegrees, int liftPosition){
        //Adjust x position until color sensor is at near edge of tape
        float adjustedX = X_TAPE_EDGE + xOffset;
        adjustPositionColor(0.5f, xOffset,initHeadingDegrees, 4, 5, 0.1f);

        bot.setPose(adjustedX, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab first stack cone
         */
        if(quadrant == Quadrant.BLUE_RIGHT || quadrant == Quadrant.RED_RIGHT) {
            driveToPosition(highSpeed, adjustedX, 16.5f, initHeadingDegrees, 1, null);
            turnToHeading(-135, 3, 6, 150);
            bot.openClaw();
            bot.setLiftPosition(liftPosition);
            sleep(200);
            driveToPosition(highSpeed, adjustedX, 10.5f, -135, 1, null);
            bot.closeClaw();
            sleep(400);
        } else{
            driveToPosition(highSpeed, adjustedX, 124.5f, initHeadingDegrees, 1, null);
            turnToHeading(45, 3, 6, 150);
            bot.openClaw();
            bot.setLiftPosition(-440);
            sleep(200);
            driveToPosition(highSpeed, adjustedX, 130.5f, 45, 1, null);
            bot.closeClaw();
            sleep(400);
        }
    }

    public void deliverMidAndLow(Quadrant quadrant, float xOffset){
        deliverMid(quadrant);
        if(quadrant == Quadrant.BLUE_RIGHT || quadrant == Quadrant.RED_RIGHT){
            driveToPosition(ultraHighSpeed, 35, 16, -90, 1,
                    new WiggleProfile(5, 0.1f, 2),
                    new Runnable() {
                        boolean done = false;
                        @Override
                        public void run() {
                            if (!done && bot.getPose().y<36) {
                                bot.setPivotPosition(OmniBot.PIVOT_GRABBING);
                                bot.setLiftPosition(OmniBot.LIFT_LOW);
                            }
                        }
                    },
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT,HeadingIndex.H_NEG_90,false,true));


            // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
            driveLine( -90, new VectorF(36,16),0,
                    highSpeed, 2, 22.5f,
                    new KalmanDistanceUpdater(null, null, null,
                            bot.frontDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
                    ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

            // Drive to stack and pick up cone
            float adjustedX = X_TAPE_EDGE + xOffset;
            driveTapeToStack45(quadrant, xOffset, -90, -440);
            bot.setLiftPosition(OmniBot.LIFT_LOW);
            sleep(200);

            /*
             * Back away from cone stack, turn, drive to junction, and drop off first stack cone on low junction
             */
            driveToPosition(highSpeed, adjustedX, 15, -135, 1, null);
            turnToHeading(90, 3, 6, 120);
            driveToPosition(highSpeed,  52.25f, 17.5f, 90, 1, null);
        } else{
            driveToPosition(ultraHighSpeed, 35, 126, -90, 1,
                    new WiggleProfile(5, 0.1f, 2),
                    new Runnable() {
                        boolean done = false;
                        @Override
                        public void run() {
                            if (!done && bot.getPose().y>107) {
                                bot.setPivotPosition(OmniBot.PIVOT_GRABBING);
                                bot.setLiftPosition(OmniBot.LIFT_LOW);
                            }
                        }
                    },
                    new PowerPlayDistUpdater(Quadrant.BLUE_LEFT,HeadingIndex.H_NEG_90,false,true));


            // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
            driveLine( -90, new VectorF(36,126), 0,
                    highSpeed, 2, 22.5f,
                    new PowerPlayDistUpdater(Quadrant.BLUE_LEFT, HeadingIndex.H_NEG_90, false, true,
                            null, d -> d>3 && d<36 && bot.getPose().x < 50),
                    ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

            //Adjust x position until color sensor is at near edge of tape
            float adjustedX = X_TAPE_EDGE + xOffset;
            driveTapeToStack45(quadrant, xOffset, -90, -440);
            bot.setLiftPosition(OmniBot.LIFT_LOW);
            sleep(200);

            /*
             * Back away from cone stack, turn, drive to junction, and drop off cone.
             */
            driveToPosition(highSpeed, adjustedX, 123.5f, 45, 1, null);
            turnToHeading(180, 3, 6, 120);
            driveToPosition(highSpeed,  52.25f, 123.5f, 180, 1, null);
        }
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);
    }

    public void deliverMid(Quadrant quadrant){
        // Raise lift to the position for middle junction
        bot.setLiftPosition(OmniBot.LIFT_MID - 100);
        if(quadrant == Quadrant.BLUE_RIGHT || quadrant == Quadrant.RED_RIGHT) {
            //Drive to mid junction; NO Kalman
            driveToPosition(highSpeed, 39, 36, -90, 1, null);
            driveToPosition(midSpeed, 36, 36, -90, 1, null);
            driveToPosition(highSpeed, 36, 50, -90, 1, null); // was midSpeed


            bot.setPivotPosition(OmniBot.PIVOT_SCORING);
            sleep(400);
            driveToPosition(lowSpeed, 38, 50.5f, -90, 1, null);
        } else{
            driveToPosition(highSpeed, 39, 105, -90, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, false)); // was vmax = 10, vmin = 4
            driveToPosition(midSpeed, 36, 105, -90, 1, null);

            driveToPosition(midSpeed, 36, 97, -90, 1, null);

            bot.setPivotPosition(OmniBot.PIVOT_SCORING);
            sleep(400);
            driveToPosition(lowSpeed, 39, 97, -90, 1, null);
        }


        // Lower lift a little, drop off cone, then raise lift again
        bot.setLiftPosition(OmniBot.LIFT_MID + 200);
        sleep(200);
        bot.openClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_MID);
        sleep(200);
    }

    public void parkFromLowJunction(Quadrant quadrant, SignalResult signalResult){
        if (quadrant == Quadrant.BLUE_RIGHT || quadrant == Quadrant.RED_RIGHT){
            driveToPosition(highSpeed, bot.getPose().x, bot.getPose().y-3, 90, 1,null);
            switch(signalResult){
                case THREE:
                    break;
                case TWO:
                case ONE:
                    driveToPosition(ultraHighSpeed, 36, bot.getPose().y, 90, 1,
                            new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, true,true));
                    float targetY = signalResult == SignalResult.ONE? 60 : 36;
                    driveToPosition(ultraHighSpeed, 36, targetY, 90, 1,
                            new WiggleProfile(3, 0.1f, 2),
                            new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, true, true,
                                    d-> d>3 && d<40 && Math.abs(d+6-bot.getPose().x)<10, d-> d>3 && d<40));
                    break;
            }
        } else {
            driveToPosition(highSpeed, 58.75f, 128.75f,180, 1, null);

            switch(signalResult){
                case THREE:
                case TWO:
                    driveToPosition(ultraHighSpeed, 35, 128.75f, 180, 1,
                            new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_180, true, true));
                    float targetY = signalResult == SignalResult.THREE? 84 : 108;
                    driveToPosition(highSpeed, 35, targetY, 180, 1, new WiggleProfile(5, 0.1f, 2),
                            new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_180, true, true,
                                    (d)-> d>3 && d<48 && Math.abs(d + 6 - bot.getPose().x) < 10, (d) -> d>3 && d>48));
                    break;
                case ONE:
                    break;
            }
        }
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
