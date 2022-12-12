package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;

@Autonomous(name = "Right Auto Two Cone Kalman")
public class RightAutoTwoConeKalman extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        // Distance updater for pose 0
        PowerPlayDistUpdater updater_0 = new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0,
                true, true);

        // Distance updater for pose 90
        PowerPlayDistUpdater updater_90 = new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90,
                true, true);

        bot.setPose(8, 36, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();

        waitForStart();

        // Get signal result
        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();

        // Raise lift to the position for low junction
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        //Drive to middle of square, turn, then drive to low junction; NO Kalman
        driveToPosition(10, 4, 12, 36, -90, 2, 1);
        turnToHeading(0, 3, 6, 60);
        driveToPosition(10, 4, 18, 43, 0, 2, 1);

        // Lower lif a little, drop off cone, then raise lift again
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Drive back to middle of starting square
        driveToPosition(10, 4, 12, 36, 0, 2, 1);

        // Drive forward, pushing signal out of the way, using Kalman for X only
        driveToPosition(10, 4, 40, 36, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Back away, to middle of square, away from signal sleeve, Kalman for X only
        driveToPosition(10, 4, 36, 36, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive toward right wall, using Kalman for X and Y
        driveToPosition(10,4,36,12,0,2,1, updater_0);



        // Drive in x direction to cone stack. TODO: May need to fix issue with the distance sensor and cone
        driveToPosition(10, 4, 58.5f, 12, 0, 2, 1);
        driveToPosition(10, 4, 58.5f, 16, 0, 2, 1);
        turnToHeading(-135, 3, 6, 60);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(300);
        driveToPosition(10, 4, 58.5f, 8, -135, 2, 1);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(300);
        driveToPosition(10, 4, 58.5f, 15, -135, 2, 1);
        turnToHeading(90, 3, 6, 60);
        driveToPosition(10, 4, 52.5f, 18, 90, 2, 1);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);
        driveToPosition(10, 4, 58.5f, 12, 90, 2, 1);
        //TODO program distance sensors to know distance from wall. Rotate to pick up cone from 5 stack. Turn and deliver on shortest poll. Park
        if(signalResult == SignalResult.ONE){
            driveToPosition(10, 4, 58.5f, 59, 90, 2, 1, updater_90);
        }else if(signalResult == SignalResult.TWO){
            driveToPosition(10, 4, 58.5f, 36, 90, 2, 1, updater_90);
        }
    }

}
