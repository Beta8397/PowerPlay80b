package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.omnibot.OmniBot;

@TeleOp(name = "testOmni", group = "test")
public class TestOmni extends LinearOpMode {
    OmniBot bot = new OmniBot();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();
        bot.setPose(0, 0, 0);
        while(opModeIsActive()){
            bot.updateOdometry();
            float pwx = gamepad1.left_stick_x/4;
            float pwy = -gamepad1.left_stick_y/4;
            float pwa = -gamepad1.right_stick_x/4;
            bot.setDrivePower(pwx, pwy, pwa);
            telemetry.addData("Pose", "X = %.1f Y = %.1f  H = %.1f",
                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));
            telemetry.update();
        }
        bot.setDrivePower(0,0, 0);
    }
}
