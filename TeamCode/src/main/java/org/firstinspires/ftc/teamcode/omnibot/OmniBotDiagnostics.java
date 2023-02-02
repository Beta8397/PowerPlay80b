/**
 * Diagnostic OpMode for use with our 2nd MechBot (with Neverest 40s, gear ratio 0.5, and Rev Hub mounted on its side
 */

package org.firstinspires.ftc.teamcode.omnibot;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

@TeleOp(name = "OmniBotDiagnostics", group = "Test")
//@Disabled
public class OmniBotDiagnostics extends LinearOpMode {

    private OmniBot bot = new OmniBot();

    boolean drivingStraight = false;
    float targetHeading = 0;

    @Override
    public void runOpMode() {

        bot.init(hardwareMap);

//        bot.getFrontLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getFrontRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getBackLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getBackRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.setPose(0, 0, 0);

        PIDFCoefficients pidfSpeed = bot.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfPosition = bot.liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("PIDFspeed", pidfSpeed.toString());
        telemetry.addData("PIDFposition", pidfPosition.toString());
        telemetry.update();


        waitForStart();

        int loopCounter = 0;
        double avgCycleMillis = 0;
        ElapsedTime cycleTimer = new ElapsedTime();
        ElapsedTime avgCycleTimer = new ElapsedTime();


        while (opModeIsActive()){
            doOneIteration();
            loopCounter++;
            double cycleMillis = cycleTimer.milliseconds();
            cycleTimer.reset();
            if (loopCounter == 100){
                loopCounter = 0;
                avgCycleMillis = avgCycleTimer.milliseconds()/100.0;
                avgCycleTimer.reset();
            }
            telemetry.addData("Cycle Time", "%.1f  Avg: %.1f", cycleMillis, avgCycleMillis);
            telemetry.update();
        }



    }

    protected void doOneIteration(){

        bot.updateOdometry();

        if (gamepad1.a) handleMotorPowers(0.2f,0,0,0);
        else if (gamepad1.x) handleMotorPowers(0,0.2f,0,0);
        else if (gamepad1.y) handleMotorPowers(0,0,0.2f,0);
        else if (gamepad1.b) handleMotorPowers(0,0,0,0.2f);
        else if (gamepad1.dpad_right) handleBotPowers(0.2f, 0, 0);
        else if (gamepad1.dpad_up) handleBotPowers(0, 0.2f, 0);
        else if (gamepad1.dpad_down) handleBotPowers(0, 0, 0.2f);
        else {
            float px = gamepad1.left_stick_x / 4;
            float py = -gamepad1.left_stick_y / 4;
            float pa = -gamepad1.right_stick_x / 4;
            handleBotPowers(px, py, pa);
        }

        telemetry.addData("Odometry", "X = %.1f  Y = %.1f  Theta = %.1f",
                bot.getPose().x, bot.getPose().y,
                Math.toDegrees(bot.getPose().theta));
        telemetry.addData("Encoders", "B %d  F %d  L %d  R %d",
                bot.bTics, bot.fTics, bot.lTics, bot.rTics);
        telemetry.addData("Speeds", "B %.0f  F %.0f,  L %.0f  R %.0f",
                bot.back.getVelocity(), bot.front.getVelocity(),
                bot.left.getVelocity(), bot.right.getVelocity());

//        telemetry.addData("DIST","F %.1f  B %.1f  R %.1f  L %.1f",bot.getFrontDistance(),
//                bot.getBackDistance(), bot.getRightDistance(), bot.getLeftDistance());

        telemetry.addData("DIST","F %.1f  B %.1f",bot.getFrontDistance(),
                bot.getBackDistance());

//        telemetry.addData("RIGHT DIST", bot.rightDist.getDistance(DistanceUnit.INCH));

        int colorInt = bot.color.getNormalizedColors().toColor();
        float[] hsv = new float[3];
        Color.colorToHSV(colorInt, hsv);
        telemetry.addData("HSV","H %.2f  S %.2f  V %.2f",hsv[0], hsv[1], hsv[2]);
        telemetry.addData("RGB","R %d  G %d  B %d", Color.red(colorInt),
                Color.green(colorInt), Color.blue(colorInt));

    }

    private void handleBotPowers(float px, float py, float pa){
        bot.setDrivePower(px, py, pa);
        telemetry.addData("Robot Power", "PX = %.2f  PY = %.2f  PA = %.2f", px, py, pa);
        telemetry.addData("Motor Powers", "B: %.2f  F: %.2f  L: %.2f  R: %.2f",
                bot.back.getPower(), bot.front.getPower(),
                bot.left.getPower(), bot.right.getPower());
    }

    private void handleMotorPowers(float p1, float p2, float p3, float p4){
        bot.back.setPower(p1);
        bot.left.setPower(p2);
        bot.front.setPower(p3);
        bot.right.setPower(p4);
        telemetry.addData("Motor Powers", "B: %.2f  F: %.2f  L: %.2f  R: %.2f", p1, p3, p2, p4);
    }

}
