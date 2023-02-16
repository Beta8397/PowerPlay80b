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
 *  Deliver preload cone to low junction. Then get one cone from stack and deliver to
 *  low junction. Then drive to the appropriate parking zone.
 *
 *  TODO: This has been changed some since the Broadalbin-Perth tournament; mainly, the
 *   turns are faster, but some drives have been changed from vMin/vMax/cp to
 *   MotionProfile. Also, wiggle syntax has changed a little (WiggleProfile). Will need
 *   to test thoroughly. Note that the original Wiggle opmode has now been retrieved,
 *   and is named "OriginalWiggle".
 *
 */

@Autonomous(name = "Wiggle2")
public class Wiggle2 extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setPose(8, 36, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);

        waitForStart();

        // Get signal result
        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();

        // Raise lift to the position for low junction
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(100);

        //Drive to middle of square, turn, then drive to low junction; NO Kalman
        driveToPosition(midSpeed, 12, 36, -90, 1, null); // was vmax = 10, vmin = 4
        turnToHeading(0, 3, 6, 120);
        driveToPosition(midSpeed, 17.0f, 41.0f, 0, 1, null);

        // Lower lift a little, drop off cone, then raise lift again
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Drive back to middle of starting square
        driveToPosition(midSpeed, 12, 35.5f, 0, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive forward, pushing signal out of the way, using Kalman for X only
        driveToPosition(highSpeed, 39, 35.5f, 0,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Back away, to middle of square, away from signal sleeve, Kalman for X only
        driveToPosition(highSpeed, 35, 35.5f, 0, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive toward right wall, using Kalman for X and Y
        driveToPosition(highSpeed, 35,13,0,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));


        // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( 0, new VectorF(36,13), 0,
                midSpeed, 2, 22.5f,
                new KalmanDistanceUpdater(null, null, null,
                        bot.rightDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        adjustPositionColor(0.5f,-1, 0, 4, 5, 0.1f);

        bot.setPose(56.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab cone
         */
        driveToPosition(midSpeed, 56.5f, 16.5f, 0, 1, null);
        turnToHeading(-135, 3, 6, 120);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(300);
        driveToPosition(midSpeed, 56.5f, 9, -135, 1, null);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(300);

        /*
         * Back away from cone stack, turn, drive to junction, and drop off cone
         */
        driveToPosition(midSpeed, 58.5f, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 120);
        driveToPosition(midSpeed,  52.25f, 15.75f, 90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to the appropriate parking location.
         */
        driveToPosition(highSpeed, 52.25f, 13, 90, 1, null);
        float speedMax = signalResult == signalResult.THREE? 10:20;
        float speedMin = signalResult == signalResult.THREE? 4:6;
        float targetX = signalResult == signalResult.THREE? 38:36;
        driveToPosition(speedMax,speedMin,targetX,13,90,4,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90, true, true));
        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.closeClaw();

        if(signalResult == SignalResult.ONE){
            driveToPosition(new MotionProfile(6, 20, 16), 36f, 58,
                    90, 1, new WiggleProfile(5, 0.1f, 2),
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT,HeadingIndex.H_90,true,true,
                            (d)->d>3 && d<48 && Math.abs(d+6-bot.getPose().x)<10,
                            (d)->d>3 && d<60));
        }else if(signalResult == SignalResult.TWO){
            driveToPosition(new MotionProfile(6, 20, 16),  36f, 36, 90, 1,
                  new WiggleProfile(5, 0.1f, 2),
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT,HeadingIndex.H_90,true,true));
        }
    }

}
