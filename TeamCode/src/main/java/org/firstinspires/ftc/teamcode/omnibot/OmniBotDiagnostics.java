/**
 * Diagnostic OpMode for use with our 2nd MechBot (with Neverest 40s, gear ratio 0.5, and Rev Hub mounted on its side
 */

package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.AngleUtil;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

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

        bot.setHeadingDegrees(0);

        waitForStart();

        while (opModeIsActive()){
            doOneIteration();
            telemetry.update();
        }



    }

    protected void doOneIteration(){

        bot.updateOdometry();

        telemetry.addData("buttons","a:%b  b:%b  x:%b  y:%b  lsy:%.2f",
                gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.left_stick_y);

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
        telemetry.addData("Encoders", "BL %d  FL %d  FR %d  BR %d",
                bot.leftBack.getCurrentPosition(), bot.leftFront.getCurrentPosition(),
                bot.rightFront.getCurrentPosition(), bot.rightBack.getCurrentPosition());
        telemetry.addData("Speeds", "BL %.0f  FL %.0f,  FR %.0f  BR %.0f",
                bot.leftBack.getVelocity(), bot.leftFront.getVelocity(),
                bot.rightFront.getVelocity(), bot.rightBack.getVelocity());

        telemetry.update();

    }

    private void handleBotPowers(float px, float py, float pa){
        bot.setDrivePower(px, py, pa);
        telemetry.addData("Robot Power", "PX = %.2f  PY = %.2f  PA = %.2f", px, py, pa);
        telemetry.addData("Motor Powers", "BL: %.2f  FL: %.2f  FR: %.2f  BR: %.2f",
                bot.leftBack.getPower(), bot.leftFront.getPower(),
                bot.rightFront.getPower(), bot.rightBack.getPower());
    }

    private void handleMotorPowers(float p1, float p2, float p3, float p4){
        bot.leftBack.setPower(p1);
        bot.leftFront.setPower(p2);
        bot.rightFront.setPower(p3);
        bot.rightBack.setPower(p4);
        telemetry.addData("Motor Powers", "BL: %.2f  FL: %.2f  FR: %.2f  BR: %.2f", p1, p2, p3, p4);
    }

}
