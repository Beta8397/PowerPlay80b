package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Predicate;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class KalmanDistanceUpdater implements KalmanMeasurementUpdater{

    protected DistanceSensor sensX = null;
    protected DistanceSensor sensY = null;
    protected Function<Float, Float> xFromDist = null;
    protected Function<Float, Float> yFromDist = null;
    protected Predicate<Float> xValid = null;
    protected Predicate<Float> yValid = null;

    private float DIST_STD_DEV_COEFF = 0.02f;
    private float DIST_VAR_COEFF = DIST_STD_DEV_COEFF * DIST_STD_DEV_COEFF;

    /* Sensor readings from prior iteration; we only want new results.
     * If current and prior distance readings are equal, it is likely that the robot is
     * stopped, and we won't apply these measurements.
     */
    private float priorXDist = 0;
    private float priorYDist = 0;

    public KalmanDistanceUpdater(){}

    public KalmanDistanceUpdater(DistanceSensor sensX, Function<Float,Float> xFromDist, Predicate<Float> xValid,
                                 DistanceSensor sensY, Function<Float,Float> yFromDist, Predicate<Float> yValid){
        this.sensX = sensX;
        this.sensY = sensY;
        this.xFromDist = xFromDist;
        this.yFromDist = yFromDist;
        this.xValid = xValid;
        this.yValid = yValid;
    }

    public Pose updatePose(Pose poseMinus, MatrixF covMinus){
        if (sensX == null && sensY == null) return poseMinus;

        float xDist = 0;
        float yDist = 0;
        boolean validX = false;
        boolean validY = false;

        if (sensX != null) {
            xDist = (float)sensX.getDistance(DistanceUnit.INCH);
            if (xDist != priorXDist) {
                validX = xValid == null || xValid.test(xDist);
                priorXDist = xDist;
            }
        }
        if (sensY != null) {
            yDist = (float)sensY.getDistance(DistanceUnit.INCH);
            if (yDist != priorYDist) {
                validY = yValid == null || yValid.test(yDist);
                priorYDist = yDist;
            }
        }

        if (sensY == null){
            if (!validX) return poseMinus;
            float xMeas = xFromDist.apply(xDist);
            float xVar = DIST_VAR_COEFF * xDist * xDist;
            return KalmanUtilities.applyXMeasurement(poseMinus, covMinus, xMeas, xVar);
        } else if (sensX == null) {
            if (!validY) return poseMinus;
            float yMeas = yFromDist.apply(yDist);
            float yVar = DIST_VAR_COEFF * yDist * yDist;
            return KalmanUtilities.applyYMeasurement(poseMinus, covMinus, yMeas, yVar);
        } else {
            if (validX && !validY) {
                float xMeas = xFromDist.apply(xDist);
                float xVar = DIST_VAR_COEFF * xDist * xDist;
                return KalmanUtilities.applyXMeasurement(poseMinus, covMinus, xMeas, xVar);
            } else if (validY && !validX) {
                float yMeas = yFromDist.apply(yDist);
                float yVar = DIST_VAR_COEFF * yDist * yDist;
                return KalmanUtilities.applyYMeasurement(poseMinus, covMinus, yMeas, yVar);
            } else if (validX) {        // to reach this point we know validX and validY are equal
                float xMeas = xFromDist.apply(xDist);
                float xVar = DIST_VAR_COEFF * xDist * xDist;
                float yMeas = yFromDist.apply(yDist);
                float yVar = DIST_VAR_COEFF * yDist * yDist;
                return KalmanUtilities.applyXYMeasurement(poseMinus, covMinus, xMeas, xVar, yMeas, yVar);
            }
            else {
                return poseMinus;
            }
        }

    }

    public KalmanDistanceUpdater reset(){
        priorXDist = 0;
        priorYDist = 0;
        return this;
    }

}
