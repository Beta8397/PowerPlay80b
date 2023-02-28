package org.firstinspires.ftc.teamcode.omnibot.old_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
@Disabled
@Autonomous(name = "Blue Right Auto Hybrid")
public class BlueRightAutoHybrid extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

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
        sleep(100);

        //Drive to middle of square, turn, then drive to low junction; NO Kalman
        driveToPosition(10, 4, 12, 36, -90, 2, 1);
        turnToHeading(0, 3, 6, 60);
        driveToPosition(10, 4, 18.5f, 42.5f, 0, 2, 1);

        // Lower lift a little, drop off cone, then raise lift again
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Drive back to middle of starting square
        driveToPosition(16, 4, 12, 35.5f, 0, 2, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive forward, pushing signal out of the way, using Kalman for X only
        driveToPosition(16, 4, 39, 35.5f, 0, 4, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Back away, to middle of square, away from signal sleeve, Kalman for X only
        driveToPosition(16, 4, 35, 35.5f, 0, 4, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        if (signalResult == SignalResult.ONE){
            driveToPosition(10, 4, 36, 59, 0, 2, 1,
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));
            return;
        }

        // If we reach this point, signal result if TWO OR THREE

        // Drive toward right wall, using Kalman for X and Y
        driveToPosition(16,4,35,13,0,4,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));


        // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine(0, new VectorF(36,13), 0,
                new MotionProfile(4, 16, 16), 2, 22.5f,
                new KalmanDistanceUpdater(null, null, null,
                        bot.rightDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        adjustPositionColor(0.5f, 0, 4, 5, 0.1f);

        bot.setPose(57.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab cone
         */
        driveToPosition(10, 4, 57.5f, 16, 0, 4, 1);
        turnToHeading(-135, 3, 6, 60);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(300);
        driveToPosition(10, 4, 57.5f, 8, -135, 4, 1);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(300);

        /*
         * Back away from cone stack, turn, drive to junction, and drop off cone
         */
        driveToPosition(10, 4, 58.5f, 15, -135, 4, 1);
        turnToHeading(90, 3, 6, 60);
        driveToPosition(10, 4, 51.25f, 16.75f, 90, 4, 1);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to the appropriate parking location.
         */
        driveToPosition(16, 4, 58.5f, 13, 90, 4, 1);
        if(signalResult == SignalResult.TWO){
            driveToPosition(16, 4, 58.5f, 36, 90, 4, 1);
        }
    }

}
