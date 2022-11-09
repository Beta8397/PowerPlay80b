package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.omnibot.OmniBot;

@TeleOp(name = "testOmni", group = "test")
public class TestOmni extends LinearOpMode {
    OmniBot bot = new OmniBot();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        int liftTarget = bot.liftMotor.getCurrentPosition();
        float clawPosition = 0;
        waitForStart();
        bot.setPose(0, 0, 0);
        while(opModeIsActive()){
            bot.updateOdometry();
            float pwxprime = gamepad1.left_stick_x/2;
            float pwyprime = -gamepad1.left_stick_y/2;
            float pwx = (pwxprime - pwyprime)/(float)Math.sqrt(2);
            float pwy = (pwxprime + pwyprime)/(float)Math.sqrt(2);
            float pwa = (gamepad1.left_trigger - gamepad1.right_trigger)/2;
            bot.setDrivePower(pwx, pwy, pwa);

            telemetry.addData("Pose", "X = %.1f Y = %.1f  H = %.1f",
                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));

            if(gamepad2.dpad_up){
                liftTarget = liftTarget - 20;
            }else if(gamepad2.dpad_down){
                liftTarget = liftTarget + 20;
            }
            liftTarget = Range.clip(liftTarget, OmniBot.LIFT_MIN, OmniBot.LIFT_MAX);
            bot.setLiftPosition(liftTarget);

            if(gamepad2.b) bot.openClaw();
            else if(gamepad2.x) bot.closeClaw();

            telemetry.addData("Lift Target", liftTarget);
            telemetry.update();
        }
        bot.setDrivePower(0,0, 0);
    }
}
