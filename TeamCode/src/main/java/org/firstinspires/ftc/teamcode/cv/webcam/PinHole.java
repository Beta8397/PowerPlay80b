package org.firstinspires.ftc.teamcode.cv.webcam;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PinHole {

    private float cameraFocalLengthPixels = 270.45f;
    private float elevationAngleDegrees = 0;
    private float elevationAngleRadians = 0;
    private float cameraHeight = 0;
    private Pose cameraPoseOnRobot = new Pose(0, 0, 0);
    private MatrixF cameraToRobotTransform = null;
    private float resX = 320;
    private float resY = 240;
    private boolean xReversed = false;
    private boolean yReversed = false;

    public PinHole(float elevationDegrees, float heightInches, Pose poseOnRobot){
        elevationAngleDegrees = elevationDegrees;
        elevationAngleRadians = (float)Math.toRadians(elevationDegrees);
        cameraHeight = heightInches;
        cameraPoseOnRobot = poseOnRobot;
        float sin = (float)Math.sin(poseOnRobot.theta);
        float cos = (float)Math.cos(poseOnRobot.theta);
        cameraToRobotTransform = new GeneralMatrixF(3, 3,
                new float[]{
                        cos, -sin, poseOnRobot.x,
                        sin, cos, poseOnRobot.y,
                        0, 0, 1
                });
    }

    public void setElevationAngle(float elevationDegrees){
        elevationAngleDegrees = elevationDegrees;
        elevationAngleRadians = (float)Math.toRadians(elevationDegrees);
    }

    public void setCameraHeight(float heightInches){
        cameraHeight = heightInches;
    }

    public void setCameraPoseOnRobot(Pose poseOnRobot){
        cameraPoseOnRobot = poseOnRobot;
        float sin = (float)Math.sin(poseOnRobot.theta);
        float cos = (float)Math.cos(poseOnRobot.theta);
        cameraToRobotTransform = new GeneralMatrixF(3, 3,
                new float[]{
                        cos, -sin, poseOnRobot.x,
                        sin, cos, poseOnRobot.y,
                        0, 0, 1
                });
    }

    public void setCameraFocalLengthPixels(float focalLengthPixels){
        cameraFocalLengthPixels = focalLengthPixels;
    }

    public void setCameraResolution(float rX, float rY){
        resX = rX;
        resY = rY;
    }

    public VectorF getSubjectPosition(float imageX, float imageY){
        float sin = (float)Math.sin(elevationAngleRadians);
        float cos = (float)Math.cos(elevationAngleRadians);

        float xPrime = imageX - resX / 2.0f;
        float yPrime = imageY - resY / 2.0f;

        if (xReversed) xPrime *= -1.0f;
        if (yReversed) yPrime *= -1.0f;

        float denom = yPrime * sin + cameraFocalLengthPixels * cos;

        float xs = cameraHeight * xPrime / denom;
        float ys = cameraHeight * (cameraFocalLengthPixels * sin - yPrime * cos) / denom;

        VectorF subjectPositionInCameraSystem = new VectorF(xs, ys, 1);
        VectorF subjectPositionInRobotSystem =
                cameraToRobotTransform.multiplied(subjectPositionInCameraSystem);

        return subjectPositionInRobotSystem;
    }


}
