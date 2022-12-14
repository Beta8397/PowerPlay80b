package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;

@Autonomous(name = "Left Auto One Cone")
public class LeftAutoOneCone extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.setPose(8, 105, -90);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);
        bot.closeClaw();

        waitForStart();

        signalResult = getSignalResult();
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Drive to low junction and drop the cone
        driveToPosition(10, 4, 15, 105, -90, 2, 1);
        driveToPosition(10, 4, 19, 98, -90, 2, 1);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        driveToPosition(10, 4, 12, 105, -90, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, false));
        driveToPosition(10, 4, 40, 105, -90, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, false));
        driveToPosition(10, 4, 36, 105, -90, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, false));

        if(signalResult == SignalResult.ONE){
            driveToPosition(10, 4, 36, 130, -90, 2, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, false, true));
        }else if(signalResult == SignalResult.THREE){
            driveToPosition(10, 4, 36, 84f, -90, 2, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, false, true));
        }
    }

}
