package org.firstinspires.ftc.teamcode.cv;



import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Pose;

/**
 * Created by FTC Team 8397 on 9/13/2019.
 */
//@Disabled
@Autonomous(name="TestVuforia", group="Test")
public class TestVuforia extends LinearOpMode {

    final OpenGLMatrix cameraLocation = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.XZX, AngleUnit.DEGREES, 90, 180, 0));

    public void runOpMode() {
        final OpenGLMatrix[] targetPositions = new OpenGLMatrix[4];

        for(int i = 0; i < 4; i++) {
            targetPositions[i] = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XYX, AngleUnit.DEGREES, 90, 0, 0));
        }

        WebcamName webcamName = hardwareMap.get(WebcamName.class,"webcam");

        VuforiaNavigator.activate("FreightFrenzy", targetPositions,
                cameraLocation, VuforiaLocalizer.CameraDirection.BACK, webcamName);

        waitForStart();

        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive()) {
            if(et.milliseconds() <= 200) {
                continue;
            } else {
                et.reset();
                OpenGLMatrix poseMatrix = null;
                for(int i = 0; i < 4; i++) {
                    poseMatrix = VuforiaNavigator.getFieldFromRobot(i);
                    if(poseMatrix != null) {
                        telemetry.addData("Target", " %d", i);
                        break;
                    }
                }
                if(poseMatrix == null) {
                    telemetry.addData("No Target Found", "");
                } else {
                    Pose pose = VuforiaNavigator.getPoseFromLocationTransform(poseMatrix);
                    telemetry.addData("Pose", " x=%.1f  y=%.1f  theta=%.1f",
                            pose.x, pose.y, pose.theta * 180.0 / Math.PI);
                }

                telemetry.update();
            }
        }

    }
}
