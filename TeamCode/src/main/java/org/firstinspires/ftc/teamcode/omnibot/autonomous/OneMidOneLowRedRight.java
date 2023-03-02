package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;

/**
 *  Deliver preload cone to mid, then two stacked cones to low junctions. Then park
 *  in the appropriate zone.
 *
 */

@Autonomous(name = "One Mid One Low Red Right")
public class OneMidOneLowRedRight extends OmniBotAuto {
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


        float xOffset = -1;
        float adjustedX = X_TAPE_EDGE + xOffset;
        deliverMidAndLow(Quadrant.BLUE_RIGHT, xOffset);

        parkFromLowJunction(Quadrant.RED_RIGHT, signalResult);

        
    }

}
