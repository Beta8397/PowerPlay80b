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

        if (sensY == null){
            float xDist = (float)sensX.getDistance(DistanceUnit.INCH);
            if (!xValid.test(xDist)) return poseMinus;
            float xMeas = xFromDist.apply(xDist);
            float xVar = DIST_VAR_COEFF * xDist * xDist;
            return KalmanUtilities.applyXMeasurement(poseMinus, covMinus, xMeas, xVar);
        } else if (sensX == null) {
            float yDist = (float)sensY.getDistance(DistanceUnit.INCH);
            if (!yValid.test(yDist)) return poseMinus;
            float yMeas = xFromDist.apply(yDist);
            float yVar = DIST_VAR_COEFF * yDist * yDist;
            return KalmanUtilities.applyXMeasurement(poseMinus, covMinus, yMeas, yVar);
        } else {
            float xDist = (float)sensX.getDistance(DistanceUnit.INCH);
            if (!xValid.test(xDist)) return poseMinus;
            float xMeas = xFromDist.apply(xDist);
            float xVar = DIST_VAR_COEFF * xDist * xDist;

            float yDist = (float)sensY.getDistance(DistanceUnit.INCH);
            if (!yValid.test(yDist)) return poseMinus;
            float yMeas = xFromDist.apply(yDist);
            float yVar = DIST_VAR_COEFF * yDist * yDist;

            return KalmanUtilities.applyXYMeasurement(poseMinus, covMinus, xMeas, xVar, yMeas, yVar);
        }

    }

}
