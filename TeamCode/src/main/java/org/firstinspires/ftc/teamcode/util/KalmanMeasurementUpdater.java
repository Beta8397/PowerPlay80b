package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

public interface KalmanMeasurementUpdater {
    /**
     * Use Kalman filter algorithm to udate estimate of robot position (X and Y components of pose)
     * based upon some set of measurements
     *
     * @param pose      Pre-existing estimate of robot pose
     * @param pMinus    Pre-existing 2x2 covariance matrix for uncertainty in pose
     * @return          New estimate of pose after measurement update
     *
     * Note that this method must also modify the covariance matrix in-place.
     */
    Pose kalmanMeasurementUpdate(Pose pose, MatrixF pMinus);
}
