package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

/**
 *  Deliver preload cone to mid, then two stacked cones to low junctions. Then park
 *  in the appropriate zone.
 *
 */

@Autonomous(name = "One Mid Two Low Blue Right")
public class OneMidTwoLowBlueRight extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setPose(8, 36, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();
        bot.grabPosition();

        waitForStart();

        // Get signal result
        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();


        float xOffset = 1;
        float adjustedX = X_TAPE_EDGE + xOffset;
        deliverMidAndLow(Quadrant.BLUE_RIGHT, xOffset);

        /*
         * Drive back to color tape, turn to position, and drive to cone stack.
         */
        xOffset = -1;
        adjustedX = X_TAPE_EDGE + xOffset;
        driveTapeToStack45(Quadrant.BLUE_RIGHT, xOffset, 90, -350);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to low junction and drop off cone.
         */

        driveToPosition(highSpeed, adjustedX, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 150);
        driveToPosition(highSpeed,  52.25f, 17.25f, 90, 1,null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to appropriate parking zone.
         *
         */

        parkFromLowJunction(Quadrant.BLUE_RIGHT, signalResult);


    }

}
