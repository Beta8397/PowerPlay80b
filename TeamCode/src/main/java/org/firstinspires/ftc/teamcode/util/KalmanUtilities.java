package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class KalmanUtilities {

    public static final MatrixF IDENTITY = new GeneralMatrixF(2, 2,
            new float[]{1,0,0,1});

    /**
     * Use Kalman Measurement Update to compute new estimated pose, and to update the robot's
     * covariance matrix, based on a new measurement of the robot's X coordinate.
     * @param poseMinus     Estimated pose prior to the measurement update.
     * @param covMinus      Robot covariance matrix prior to the measurement update.
     * @param xMeas         New measurement of robot X coordinate.
     * @param xMeasVar      Variance of the new measurement.
     * @return              Updated Pose estimate.
     * 
     * Note: This method also updates the covariance matrix poseMinus (from P-minus to P)
     */
    public static Pose applyXMeasurement(Pose poseMinus, MatrixF covMinus, float xMeas,
                                         float xMeasVar) {
        /*
         * In this case the matrix H = [1 0], so that the product of H and the robot's pose
         * ( [X Y]-Transposed ) would give a 1x1 measurement vector z = [X].
         * Matrix R is 1x1, and contains the variance of the X measurement.
         */
        MatrixF H = new GeneralMatrixF(1, 2, new float[]{1, 0});
        MatrixF R = new GeneralMatrixF(1,1, new float[]{xMeasVar});
        VectorF z = new VectorF(xMeas);
        return applyKalman(poseMinus, covMinus, z, H, R);
    }

    /**
     * Use Kalman Measurement Update to compute new estimated pose, and to update the robot's
     * covariance matrix, based on a new measurement of the robot's Y coordinate.
     * @param poseMinus     Estimated pose prior to the measurement update.
     * @param covMinus      Robot covariance matrix prior to the measurement update.
     * @param yMeas         New measurement of robot Y coordinate.
     * @param yMeasVar      Variance of the new measurement.
     * @return              Updated Pose estimate.
     *
     * Note: This method also updates the covariance matrix poseMinus (from P-minus to P)
     */
    public static Pose applyYMeasurement(Pose poseMinus, MatrixF covMinus, float yMeas,
                                         float yMeasVar){
        /*
         * In this case the matrix H = [0 1], so that the product of H and the robot's pose
         * ( [X Y]-Transposed ) would give a 1x1 measurement vector z = [Y].
         * Matrix R is 1x1, and contains the variance of the Y measurement.
         */
        MatrixF H = new GeneralMatrixF(1, 2, new float[]{0, 1});
        MatrixF R = new GeneralMatrixF(1,1, new float[]{yMeasVar});
        VectorF z = new VectorF(yMeas);
        return applyKalman(poseMinus, covMinus, z, H, R);
    }
    
    /**
     * Use Kalman Measurement Update to compute new estimated pose, and to update the robot's
     * covariance matrix, based on new measurements of the robot's X and Y coordinates.
     * @param poseMinus     Estimated pose prior to the measurement update.
     * @param covMinus      Robot covariance matrix prior to the measurement update.
     * @param xMeas         New measurement of robot X coordinate.
     * @param xMeasVar      Variance of the new X measurement.                
     * @param yMeas         New measurement of robot Y coordinate.
     * @param yMeasVar      Variance of the new Y measurement.
     * @return              Updated Pose estimate.
     *
     * Note: This method also updates the covariance matrix poseMinus (from P-minus to P)
     */
    public static Pose applyXYMeasurement(Pose poseMinus, MatrixF covMinus, float xMeas,
                                          float xMeasVar, float yMeas, float yMeasVar){
        /*
         * In this case, matrix H is the identity matrix, because the measurement vector, 
         * so that the measurement vector z is 2x1, and contains the X and Y measurements.
         * Matrix R is 2x2, diagonal, and contains the variances of the X and Y measurements.
         */
        MatrixF H = MatrixF.identityMatrix(2);
        MatrixF R = new GeneralMatrixF(2,2,
                new float[]{xMeasVar,0,0,yMeasVar});
        VectorF z = new VectorF(xMeas, yMeas);
        return applyKalman(poseMinus, covMinus, z, H, R);
    }

    /**
     * Use Kalman Measurement Update to calculate new estimated pose and update the robot's
     * covariance matrix (P)
     * @param poseMinus Robot pose prior to measurement update
     * @param covMinus  Robot 2x2 covariance matrix prior to measurement update
     * @param z  Measurement vector (this could be [x], [y], or [x,y]-transposed
     * @param H  Matrix that converts from [x,y]-transposed to z. This could be [1,0], [0,1], or I
     * @param R  Covariance matrix for the measurement itself.
     * @return  Updated Pose
     *
     * Note: this method also updates the robot covariance matrix, covMinus (i.e., updates
     * from P-minus to P)
     */
    public static Pose applyKalman(Pose poseMinus, MatrixF covMinus, VectorF z, MatrixF H, MatrixF R){
        MatrixF K = covMinus.multiplied(H.transposed()).multiplied(
                H.multiplied(covMinus.multiplied(H.transposed())).added(R).inverted());
        VectorF xMinus = new VectorF(poseMinus.x, poseMinus.y);
        VectorF x = xMinus.added(K.multiplied(z.subtracted(H.multiplied(xMinus))));
        MatrixF cov = MatrixF.identityMatrix(2).subtracted(K.multiplied(H)).multiplied(covMinus);
        covMinus.put(0,0, cov.get(0,0));
        covMinus.put(0,1, cov.get(0,1));
        covMinus.put(1,0, cov.get(1,0));
        covMinus.put(1,1, cov.get(1,1));
        return new Pose(x.get(0), x.get(1), poseMinus.theta);
    }

    /**
     * Estimate the change in the 2x2 covariance matrix (due to PROCESS) for a holonomic robot with
     * independent movements along its forward and right axes.
     * @param dXR           Increment of robot motion along its right axis
     * @param dYR           Increment of robot motion along its forward axis
     * @param varXR         Variance for dXR
     * @param varYR         Variance for dYR
     * @param varTheta      Variance for heading
     * @param sin           Sine of heading
     * @param cos           Cosine of heading
     * @return              Matrix
     */
    public static MatrixF QMatrix(float dXR, float dYR, float varXR, float varYR,
                                  float varTheta, float sin, float cos){

        MatrixF result = new GeneralMatrixF(2,2);
        float cosSqr = cos*cos;
        float sinSqr = sin*sin;
        float sinCos = sin*cos;
        float xFactor = varXR + dXR*dXR;
        float yFactor = varYR + dYR*dYR;
        float dXRdYR = dXR * dYR;

        float varX = varXR*sinSqr + varYR*cosSqr
                + varTheta * (xFactor*cosSqr + yFactor*sinSqr - 2*dXRdYR*sinCos);
        float varY = varXR*cosSqr + varYR*sinSqr
                + varTheta * (xFactor*sinSqr + yFactor*sinSqr + 2*dXRdYR*sinCos);
        float covXY = (varYR - varXR + varTheta*(xFactor - yFactor))*sinCos
                + dXRdYR * varTheta * (cosSqr - sinSqr);

        result.put(0,0, varX);
        result.put(1,1, varY);
        result.put(0,1, covXY);
        result.put(1,0, covXY);

        return result;

    }
}
