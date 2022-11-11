package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "testOmni", group = "test")
public class TestOmni extends LinearOpMode {

    OmniBot bot = new OmniBot();

    ButtonToggle leftBump2Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {return false;} };
    boolean adjustLiftMode = false;

    ButtonToggle b2Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {return gamepad2.b;}};

    ButtonToggle x2Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {return gamepad2.x;}};

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        int liftTarget = bot.liftMotor.getCurrentPosition();
        float clawPosition = 0;
        waitForStart();
        bot.setPose(0, 0, 0);
        while(opModeIsActive()){

            // Obtain and display new pose
            bot.updateOdometry();
            telemetry.addData("Pose", "X = %.1f Y = %.1f  H = %.1f",
                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));

            // For teleop, treat chassis as a diamond (not a square)
            float pwxprime = gamepad1.left_stick_x/2;
            float pwyprime = -gamepad1.left_stick_y/2;
            float pwx = (pwxprime - pwyprime)/(float)Math.sqrt(2);
            float pwy = (pwxprime + pwyprime)/(float)Math.sqrt(2);
            float pwa = (gamepad1.left_trigger - gamepad1.right_trigger)/2;
            bot.setDrivePower(pwx, pwy, pwa);

            // Handle AdjustLiftMode
            if (leftBump2Toggle.update()){
                adjustLiftMode = true;
            } else if (adjustLiftMode && !gamepad2.left_bumper){
                adjustLiftMode = false;
                bot.resetLiftEncoder();
            }

            // Handle lift
            if(gamepad2.dpad_up){
                liftTarget = liftTarget - 20;
            }else if(gamepad2.dpad_down){
                liftTarget = liftTarget + 20;
            }

            if (!adjustLiftMode) {
                liftTarget = Range.clip(liftTarget, OmniBot.LIFT_MIN, OmniBot.LIFT_MAX);
            }

            bot.setLiftPosition(liftTarget);

            // Handle claw
            if(b2Toggle.update()) {
                bot.openClaw();
                telemetry.speak("Open Claw");
            }

            if(x2Toggle.update()) {
                bot.closeClaw();
                telemetry.speak("Close Claw");
            }

            telemetry.addData("Lift Target", liftTarget);
            telemetry.update();
        }
        bot.setDrivePower(0,0, 0);
    }
}
