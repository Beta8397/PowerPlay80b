package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

/**
 *  Deliver preload cone to mid junction, then two stack cones to low junction,
 *  then park.
 *
 */

@Autonomous(name = "One Mid Park Left")
public class OneMidParkLeft extends OmniBotAuto {
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

        deliverMid(Quadrant.BLUE_LEFT);

        driveToPosition(midSpeed, 34, bot.getPose().y, -90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);
        sleep(500);

        float targetY = YMAX - 13;
        if(signalResult == SignalResult.TWO){
            targetY = YMAX - 36;
        } else if(signalResult == SignalResult.THREE){
            targetY = YMAX - 58;
        }

        driveToPosition(midSpeed, 36, targetY, -90, 1,
                new WiggleProfile(5, 0.1f, 2),
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, true,
                        (d) -> d>3 && d<36 && Math.abs(d + 6 - bot.getPose().x) <10,
                        (d) -> d>3 && d<40 && Math.abs(d + 6 - bot.getPose().y) < 10));

    }

}

