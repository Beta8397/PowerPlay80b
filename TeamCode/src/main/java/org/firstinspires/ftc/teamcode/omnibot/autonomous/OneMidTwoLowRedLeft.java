package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;

/**
 *  Deliver preload cone to mid junction, then two stack cones to low junction,
 *  then park.
 */

@Autonomous(name = "One Mid Two Low Red Left")
public class OneMidTwoLowRedLeft extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bot.setPose(8, 105, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();
        bot.grabPosition();

        waitForStart();

        totalTime.reset();

        // Get signal result
        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();


        /*
         * Deliver cone to mid, drive to stack, get cone and deliver it to low
         */

        float xOffset = 2;
        float adjustedX = X_TAPE_EDGE + xOffset;
        deliverMidAndLow(Quadrant.BLUE_LEFT, xOffset);

        /*
         * Drive back along tape and grab a cone at stack
         */
        xOffset = 1.5f;
        adjustedX = X_TAPE_EDGE + xOffset;
        driveTapeToStack45(Quadrant.BLUE_LEFT, xOffset,180, -330);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to low junction and drop off cone.
         */

        driveToPosition(highSpeed, adjustedX, 123f, 45, 1, null);
        turnToHeading(180, 3, 6, 150);
        driveToPosition(highSpeed,  53.25f, 124.25f, 180, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to appropriate parking zone.
         */

        parkFromLowJunction(Quadrant.BLUE_LEFT, signalResult);

    }

}

