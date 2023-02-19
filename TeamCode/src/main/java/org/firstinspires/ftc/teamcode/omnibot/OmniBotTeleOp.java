package org.firstinspires.ftc.teamcode.omnibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "OmniBot TeleOp", group = "Comp")
public class OmniBotTeleOp extends LinearOpMode {

    OmniBot bot = new OmniBot();

    public enum DriveSpeed {SLOW,NORMAL,HIGH}
    private DriveSpeed speed = DriveSpeed.NORMAL;

    ButtonToggle leftBump2Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad2.left_bumper;
        }
    };
    boolean adjustLiftMode = false;

    ButtonToggle x2Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad2.x;
        }
    };

    ButtonToggle a1Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle y1Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.y;
        }
    };

    ButtonToggle dPadDown1Toggle = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_down;
        }
    };

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);

        int liftTarget = bot.liftMotor.getCurrentPosition();

        boolean clawClosed = false;
        boolean pivotScoring = false;

        bot.setClawPosition(OmniBot.CLAW_OPEN_TELE);
        bot.setPivotPosition(OmniBot.PIVOT_TELE);


        speed = DriveSpeed.NORMAL;

        waitForStart();
        bot.setPose(0, 0, 0);
        while (opModeIsActive()) {

            // Obtain and display new pose
            bot.updateOdometry();
            telemetry.addData("Pose", "X = %.1f Y = %.1f  H = %.1f",
                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));

            boolean a1Toggled = a1Toggle.update();
            boolean dPadDown1Toggled = dPadDown1Toggle.update();

            switch (speed) {
                case SLOW:
                    if (a1Toggled) speed = DriveSpeed.HIGH;
                    else if (dPadDown1Toggled) speed = DriveSpeed.NORMAL;
                    break;
                case NORMAL:
                    if (a1Toggled) speed = DriveSpeed.HIGH;
                    else if (dPadDown1Toggled) speed = DriveSpeed.SLOW;
                    break;
                case HIGH:
                    if (a1Toggled) speed = DriveSpeed.NORMAL;
                    else if (dPadDown1Toggled) speed = DriveSpeed.SLOW;
            }

            // Treat chassis as a diamond
            float pwxprime = gamepad1.left_stick_x / 2.4f;
            float pwyprime = -gamepad1.left_stick_y / 2.4f;
            float pwx = (pwxprime - pwyprime) / (float) Math.sqrt(2);
            float pwy = (pwxprime + pwyprime) / (float) Math.sqrt(2);

            //Treat chassis as a square
//            float pwx = +gamepad1.left_stick_y/2;
//            float pwy = +gamepad1.left_stick_x/2;

            float pwa = (gamepad1.left_trigger - gamepad1.right_trigger) / 2.8f;
            float speedMultiplier = speed == DriveSpeed.NORMAL? 1.2f : speed == DriveSpeed.HIGH? 1.5f : 1.0f;
            pwx *= speedMultiplier;
            pwy *= speedMultiplier;
            pwa *= speedMultiplier;

            telemetry.addData("speed", speed);

            bot.setDrivePower(pwx, pwy, pwa);

            // Handle AdjustLiftMode
            if (leftBump2Toggle.update()) {
                adjustLiftMode = true;
            } else if (adjustLiftMode && !gamepad2.left_bumper) {
                adjustLiftMode = false;
                bot.resetLiftEncoder();
            }

            // Handle lift
            if (gamepad2.dpad_up) {
                liftTarget = liftTarget - 30;
            } else if (gamepad2.dpad_down) {
                liftTarget = liftTarget + 30;
            } else if (gamepad2.y) {
                liftTarget = OmniBot.LIFT_HIGH;
            } else if (gamepad2.b) {
                liftTarget = OmniBot.LIFT_MID;
            } else if (gamepad2.a) {
                liftTarget = OmniBot.LIFT_LOW;
            }

            if (!adjustLiftMode) {
                liftTarget = Range.clip(liftTarget, OmniBot.LIFT_MIN, OmniBot.LIFT_MAX);
            }

            bot.setLiftPosition(liftTarget);

            telemetry.addData("Lift", "tics = %d  target = %d",
                    bot.liftMotor.getCurrentPosition(), liftTarget);

            // Handle claw
            if (x2Toggle.update()) {
                clawClosed = !clawClosed;
                if (clawClosed) {
                    bot.closeClaw();
                } else {
                    bot.setClawPosition(OmniBot.CLAW_OPEN_TELE);
                }
            }

            // Handle Pivot

            if (y1Toggle.update()){
                pivotScoring = !pivotScoring;
                if (pivotScoring){
                    bot.scorePosition();
                } else {
                    bot.setPivotPosition(OmniBot.PIVOT_TELE);
                }
            }


            telemetry.update();

        }
        bot.setDriveSpeed(0,0,0);
    }
}
