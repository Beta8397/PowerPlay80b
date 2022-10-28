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
        bot.setHeadingDegrees(45);
        while(opModeIsActive()){
            float pwx = gamepad1.left_stick_x;
            float pwy = -gamepad1.left_stick_y;
            float pwa = -gamepad1.right_stick_x;
            bot.setDrivePower(pwx, pwy, pwa);
            telemetry.addData("Heading",
                    Math.toDegrees(bot.getHeading()));
            telemetry.update();
        }
        bot.setDrivePower(0,0, 0);
    }
}
