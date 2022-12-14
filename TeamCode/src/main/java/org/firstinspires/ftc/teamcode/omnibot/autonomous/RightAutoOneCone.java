package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;

@Autonomous(name = "Right Auto One Cone")
public class RightAutoOneCone extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.setPose(8, 36, -90);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);
        bot.closeClaw();
        waitForStart();
        signalResult = getSignalResult();
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        //Drive to low junction and drop off pre-loaded cone
        driveToPosition(10, 4, 12, 36, -90, 2, 1);
        turnToHeading(0, 3, 6, 60);
        driveToPosition(10, 4, 18.5f, 42.5f, 0, 2, 1); // was 18, 43
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Back away from low junction, push signal away, and park
        driveToPosition(10, 4, 12, 35.5f, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));
        driveToPosition(10, 4, 40, 35.5f, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));
        driveToPosition(10, 4, 36, 35.5f, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));
        if(signalResult == SignalResult.ONE){
            driveToPosition(10, 4, 36, 59, 0, 2, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));
        }else if(signalResult == SignalResult.THREE){
            driveToPosition(10, 4, 36, 8, 0, 2, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));
        }
    }

}
