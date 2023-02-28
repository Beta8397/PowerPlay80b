package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

@Autonomous(name = "All Levels Blue Right")
public class AllLevelsBlueRight extends OmniBotAuto {
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
        deliverMidAndLow(Quadrant.RED_RIGHT, xOffset);

        /*
         * Drive back to color tape, turn to position, and drive to cone stack.
         */
        xOffset = -1;
        adjustedX = X_TAPE_EDGE + xOffset;
        driveTapeToStack45(Quadrant.BLUE_RIGHT, xOffset, 90, -350);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drop cone on high jct
         */


        driveToPosition(midSpeed, adjustedX, 15, -135, 1, null);
        turnToHeading(-90, 3, 6, 150);

        driveToPosition(highSpeed, 60, 50.5f, -90, 1, // was safeSpeed
                new WiggleProfile(5, 0.1f, 2),
                new Runnable() {
                    boolean liftUp = false;
                    @Override
                    public void run() {
                        if (!liftUp && bot.getPose().y > 30) {
                            bot.setLiftPosition(OmniBot.LIFT_HIGH);
                            liftUp = true;
                        }
                    }
                },
        null); //added wiggle
        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        driveToPosition(safeSpeed,  63, 51.5f, -90, 1, null);
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_HIGH + 300);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_HIGH -100);
        sleep(200);
        driveToPosition(midSpeed,  60, 50.5f, -90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);


        switch(signalResult){
            case THREE:
                driveToPosition(highSpeed, bot.getPose().x-2, 15, -90, 1,null);
                break;
            case TWO:
                driveToPosition(ultraHighSpeed, bot.getPose().x-2, 38, -90, 1,
                        new WiggleProfile(3, 0.1f, 2), null);
                break;
            case ONE:
                driveToPosition(lowSpeed, bot.getPose().x - 3.5f, 61, -90, 1,
                new WiggleProfile(3, 0.1f, 2), null);
                break;
        }


        while(opModeIsActive()){
            continue;
        }

    }

}

