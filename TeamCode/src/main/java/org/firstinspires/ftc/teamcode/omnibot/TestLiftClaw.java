package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestLiftClaw")
public class TestLiftClaw extends LinearOpMode {
    OmniBot bot = new OmniBot();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        int liftTarget = bot.liftMotor.getCurrentPosition();
        float clawPosition = 0;
        bot.setClawPosition(clawPosition);
        waitForStart();
        while(opModeIsActive()){
            int liftCurrent = bot.liftMotor.getCurrentPosition();
            telemetry.addData("Lift Tics", liftCurrent);
            if(gamepad1.dpad_up){
                liftTarget = liftTarget - 10;
            }else if(gamepad1.dpad_down){
                liftTarget = liftTarget + 10;
            }
            liftTarget = Range.clip(liftTarget, OmniBot.LIFT_MIN, OmniBot.LIFT_MAX);
            bot.setLiftPosition(liftTarget);
            if(gamepad1.dpad_right){
                clawPosition = clawPosition + 0.01f;
            } else if(gamepad1.dpad_left){
                clawPosition = clawPosition - 0.01f;
            }
            clawPosition = Range.clip(clawPosition, 0, 1);
            bot.setClawPosition(clawPosition);
            telemetry.addData("Lift Target", liftTarget);
            telemetry.addData("Claw Position", clawPosition);
            telemetry.addData("Hello Kitty","");
            telemetry.update();
        }
    }
}
