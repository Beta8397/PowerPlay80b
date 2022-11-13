package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.Pose;


public class OmniBot {
    DcMotorEx back;
    DcMotorEx front;
    DcMotorEx left;
    DcMotorEx right;
    BNO055Enhanced imu;
    public  DcMotorEx liftMotor;
    Servo clawServo;

    float headingOffset = 0;

    int lTics = 0;
    int fTics = 0;
    int rTics = 0;
    int bTics = 0;

    private Pose pose = new Pose(0, 0, 0);

    public static final float TICS_PER_RADIAN = 276.1f; // Empiric Measurement
    public static final float TICS_PER_INCH = 44.2f;    // Empiric Measurement
    public static final float MAX_TICS_PER_SEC = 2500;
    public static final int LIFT_MIN = -2600;
    public static final int LIFT_MAX = 0;
    public static final float CLAW_OPEN = 0.62f;
    public static final float CLAW_CLOSED = 0.32f;

    public void init(HardwareMap hwmap){
        back = hwmap.get(DcMotorEx.class, "back");
        right = hwmap.get(DcMotorEx.class, "right");
        front = hwmap.get(DcMotorEx.class, "front");
        left = hwmap.get(DcMotorEx.class, "left");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        float pL = py + 0 - pa;
        float pR = py + 0 + pa;
        float pB = 0 + px + pa;
        float pF = 0 + px - pa;
        float max = Math.max(Math.abs(pL), Math.abs(pR));
        max = Math.max(max, Math.abs(pB));
        max = Math.max(max, Math.abs(pF));
        if(max > 1){
            pL = pL/max;
            pR /= max;
            pB /= max;
            pF /= max;
        }
        back.setPower(pB);
        front.setPower(pF);
        right.setPower(pR);
        left.setPower(pL);
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
       int lCurrTics = left.getCurrentPosition();
       int rCurrTics = right.getCurrentPosition();
       int fCurrTics = front.getCurrentPosition();
       int bCurrTics = back.getCurrentPosition();
       int lNew = lCurrTics - lTics;
       int rNew = rCurrTics - rTics;
       int fNew = fCurrTics - fTics;
       int bNew = bCurrTics - bTics;
       lTics = lCurrTics;
       rTics = rCurrTics;
       fTics = fCurrTics;
       bTics = bCurrTics;

       float dYR = 0.5f * (lNew + rNew) / TICS_PER_INCH;
       float dXR = 0.5f * (fNew + bNew) / TICS_PER_INCH;

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
        lTics = left.getCurrentPosition();
        rTics = right.getCurrentPosition();
        fTics = front.getCurrentPosition();
        bTics = back.getCurrentPosition();
    }

    public Pose getPose(){
        return pose;
    }

    public void setClawPosition(float pos){
        clawServo.setPosition(pos);
    }

    public void openClaw(){setClawPosition(CLAW_OPEN);}

    public void closeClaw(){setClawPosition(CLAW_CLOSED);}

    public void setLiftPosition(int tics){
        liftMotor.setTargetPosition(tics);
        liftMotor.setPower(1f);
    }

    public void resetLiftEncoder(){
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
