package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

/**
 *  Deliver preload cone to mid junction, then two stack cones to low junction,
 *  then park.
 */

@Autonomous(name = "One Mid Two Low Left")
public class OneMidTwoLowLeft extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setPose(8, 105, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();
        bot.grabPosition();

        waitForStart();

        // Get signal result
        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();


        /*
         * Deliver cone to mid, drive to stack, get cone and deliver it to low
         */

        float xOffset = 3;
        float adjustedX = X_TAPE_EDGE + xOffset;
        deliverMidAndLow(Quadrant.BLUE_LEFT, xOffset);

        /*
         * Drive back along tape and grab a cone at stack
         */
        driveTapeToStack45(Quadrant.BLUE_LEFT, xOffset,180, -350);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to low junction and drop off cone.
         */

        driveToPosition(highSpeed, 59.5f, 123f, 45, 1, null);
        turnToHeading(180, 3, 6, 150);
        driveToPosition(highSpeed,  53.25f, 124.25f, 180, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to appropriate parking zone.
         */

        parkFromLowJunction(Quadrant.BLUE_LEFT, signalResult);

    }

}

