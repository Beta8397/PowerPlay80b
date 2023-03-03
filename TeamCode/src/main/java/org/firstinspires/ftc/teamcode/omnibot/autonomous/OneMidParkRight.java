package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

/**
 *  Deliver preload cone to mid, then two stacked cones to low junctions. Then park
 *  in the appropriate zone.
 *
 */

@Autonomous(name = "One Mid Park Right")
public class OneMidParkRight extends OmniBotAuto {
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


        deliverMid(Quadrant.BLUE_RIGHT);

        driveToPosition(midSpeed, 34, bot.getPose().y, -90, 1, null);

        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);
        sleep(500);

        float targetY;
        if(signalResult == SignalResult.TWO){
            targetY = 36;
        } else if(signalResult == SignalResult.THREE){
            targetY = 13;
        } else targetY = 60;

        driveToPosition(midSpeed, 36, targetY, -90, 1,
                new WiggleProfile(5, 0.1f, 2),
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_NEG_90, true, true,
                        (d) -> d>3 && d<36 && Math.abs(d + 6 - bot.getPose().x) < 10,
                        (d) -> d>3 && d<40 && Math.abs(d + 6 - bot.getPose().y) < 10));

        while(opModeIsActive()){
            telemetry.addData("Pose", "X = %.1f  Y = %.1f  TH = %.1f", bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));
            telemetry.addData("Dist", "Front = %.1f", bot.frontDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
