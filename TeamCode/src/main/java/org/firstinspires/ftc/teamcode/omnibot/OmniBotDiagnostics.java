/**
 * Diagnostic OpMode for use with our 2nd MechBot (with Neverest 40s, gear ratio 0.5, and Rev Hub mounted on its side
 */

package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        bot.setPose(0, 0, 0);

        waitForStart();

        while (opModeIsActive()){
            doOneIteration();
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
                bot.back.getCurrentPosition(), bot.front.getCurrentPosition(),
                bot.left.getCurrentPosition(), bot.right.getCurrentPosition());
        telemetry.addData("Speeds", "B %.0f  F %.0f,  L %.0f  R %.0f",
                bot.back.getVelocity(), bot.front.getVelocity(),
                bot.left.getVelocity(), bot.right.getVelocity());

        telemetry.update();

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
