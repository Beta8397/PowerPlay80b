package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Pose;


public class OmniBot {
    DcMotorEx leftBack;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    BNO055Enhanced imu;
    DcMotorEx liftMotor;
    Servo clawServo;

    float headingOffset = 0;

    int blTics = 0;
    int flTics = 0;
    int brTics = 0;
    int frTics = 0;

    private Pose pose = new Pose(0, 0, 0);

    public static final float TICS_PER_RADIAN = 537.6f * 13/(8 * (float)Math.PI);
    public static final float TICS_PER_INCH = 537.6f / (4 * (float)Math.sqrt(2) * (float)Math.PI);
    public static final float MAX_TICS_PER_SEC = 2500;

    public void init(HardwareMap hwmap){
        leftBack = hwmap.get(DcMotorEx.class, "left_back");
        rightBack = hwmap.get(DcMotorEx.class, "right_back");
        leftFront = hwmap.get(DcMotorEx.class, "left_front");
        rightFront = hwmap.get(DcMotorEx.class, "right_front");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hwmap.get(BNO055Enhanced.class, "imu");
        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.axesMap = BNO055Enhanced.AxesMap.XZY;
        parameters.axesSign = BNO055Enhanced.AxesSign.PPN;
        parameters.accelUnit = BNO055Enhanced.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055Enhanced.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";

        liftMotor = hwmap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServo = hwmap.get(Servo.class, "clawServo");

        imu.initialize(parameters);
    }
    public void setDrivePower(float px, float py, float pa) {
        float pBL = -px + py - pa;
        float pFL = +px + py - pa;
        float pBR = +px + py + pa;
        float pFR = -px + py + pa;
        float max = Math.max(Math.abs(pBL), Math.abs(pFL));
        max = Math.max(max, Math.abs(pBR));
        max = Math.max(max, Math.abs(pFR));
        if(max > 1){
            pBL = pBL/max;
            pFL /= max;
            pBR /= max;
            pFR /= max;
        }
        leftBack.setPower(pBL);
        leftFront.setPower(pFL);
        rightBack.setPower(pBR);
        rightFront.setPower(pFR);
    }

    public void setDriveSpeed(float vx, float vy, float va){
        float px = vx * TICS_PER_INCH / MAX_TICS_PER_SEC;
        float py = vy * TICS_PER_INCH / MAX_TICS_PER_SEC;
        float pa = va * TICS_PER_RADIAN / MAX_TICS_PER_SEC;
        setDrivePower(px, py, pa);
    }


    public float getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.RADIANS);
        float result = AngleUtil.normalizeRadians(headingOffset + angles.firstAngle);
        return result;
    }

    public void setHeadingDegrees(float headingDegrees){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.RADIANS);
        headingOffset = AngleUtil.normalizeRadians((float)Math.toRadians(headingDegrees) - angles.firstAngle);
    }

    public Pose updateOdometry(){
       int blCurrTics = leftBack.getCurrentPosition();
       int brCurrTics = rightBack.getCurrentPosition();
       int flCurrTics = leftFront.getCurrentPosition();
       int frCurrTics = rightFront.getCurrentPosition();
       int blNew = blCurrTics - blTics;
       int brNew = brCurrTics - brTics;
       int flNew = flCurrTics - flTics;
       int frNew = frCurrTics - frTics;
       blTics = blCurrTics;
       brTics = brCurrTics;
       flTics = flCurrTics;
       frTics = frCurrTics;

       float dYR = 0.25f * (blNew + frNew + brNew + flNew) / TICS_PER_INCH;
       float dXR = 0.25f * (flNew - frNew - blNew + brNew) / TICS_PER_INCH;

       float newHeading = getHeading();
       float headingChange = AngleUtil.normalizeRadians(newHeading - pose.theta);
       float avgHeading = AngleUtil.normalizeRadians(pose.theta + headingChange / 2);
       float sin = (float)Math.sin(avgHeading);
       float cos = (float)Math.cos(avgHeading);
       float x = pose.x + dXR * sin + dYR * cos;
       float y = pose.y - dXR * cos + dYR * sin;
       pose = new Pose(x, y, newHeading);
       return pose;
    }

    public void setPose(float x, float y, float headingDegrees){
        setHeadingDegrees(headingDegrees);
        pose = new Pose(x, y, (float)Math.toRadians(headingDegrees));
        blTics = leftBack.getCurrentPosition();
        brTics = rightBack.getCurrentPosition();
        flTics = leftFront.getCurrentPosition();
        frTics = rightFront.getCurrentPosition();
    }

    public Pose getPose(){
        return pose;
    }

    public void setClawPosition(float pos){
        clawServo.setPosition(pos);
    }

    public void setLiftPosition(int tics){
        liftMotor.setTargetPosition(tics);
        liftMotor.setPower(0.4f);
    }
}
