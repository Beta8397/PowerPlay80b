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

@Autonomous(name = "WiggleTwoLow")
public class WiggleTwoLow extends OmniBotAuto {
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

        // Raise lift to the position for middle junction
        bot.setLiftPosition(OmniBot.LIFT_MID - 100);
        //sleep(100);

        //Drive to mid junction; NO Kalman
        driveToPosition(highSpeed, 39, 36, -90, 1, null); // was vmax = 10, vmin = 4
        driveToPosition(midSpeed, 36, 36, -90, 1, null);

        //turnToHeading(0, 3, 6, 150);
        driveToPosition(midSpeed, 36, 50, -90, 1, null);

        //new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_NEG_90, true, false, (d) -> d>3 && d<40, null));

        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        sleep(400);
        driveToPosition(lowSpeed, 38, 50.5f, -90, 1, null);

        //driveToPosition(highSpeed, 38, 48, -90, 1, null);


        // Lower lift a little, drop off cone, then raise lift again
        bot.setLiftPosition(OmniBot.LIFT_MID + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_MID);
        sleep(200);


        driveToPosition(highSpeed, 36, 36, -90, 1, null);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);

        // Drive back to middle of starting square
        // driveToPosition(ultraHighSpeed, 12, 35.5f, 0, 1,
        //  new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive forward, pushing signal out of the way, using Kalman for X only
        //driveToPosition(ultraHighSpeed, 36, 35.5f, 0,1,
        //  new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Back away, to middle of square, away from signal sleeve, Kalman for X only
        //driveToPosition(ultraHighSpeed, 34, 35.5f, -90, 1,
        //       new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        bot.setLiftPosition(OmniBot.LIFT_LOW);

        // Drive toward right wall, using Kalman for X and Y
        driveToPosition(ultraHighSpeed, 35, 16, -90, 1, null);


        // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( -90, new VectorF(36,16),0,
                highSpeed, 2, 22.5f,
                new KalmanDistanceUpdater(null, null, null,
                        bot.frontDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        adjustPositionColor(0.5f, -1,-90, 4, 5, 0.1f);

        bot.setPose(56.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab first stack cone
         */
        driveToPosition(highSpeed, 57.5f, 16.5f, -90, 1, null);
        turnToHeading(-135, 3, 6, 150);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(200);
        driveToPosition(highSpeed, 57.5f, 10.5f, -135, 1, null);
        bot.closeClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Back away from cone stack, turn, drive to junction, and drop off first stack cone on low junction
         */
        driveToPosition(highSpeed, 58.5f, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 120);
        driveToPosition(highSpeed,  52.25f, 17.5f, 90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to color tape, turn to position, and drive to cone stack.
         */
//
        adjustPositionColor(0.5f,90, 4, 5, 0.1f);
        turnToHeading(-135, 3, 6, 120);
        adjustPositionColor(0.5f,-1,-135, 4, 5, 0.1f);
        bot.setPose(56.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /*
         * Pick up second cone from stack
         */

        bot.openClaw();
        bot.setLiftPosition(-350);
        sleep(300);
        driveToPosition(highSpeed, 57.5f, 10.5f, -135, 1, null);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drop cone on low jct
         */


        driveToPosition(highSpeed, 58.5f, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 150);
        driveToPosition(highSpeed, 52.25f, 17.5f, 90, 1, null);


                //new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true));
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);




        driveToPosition(highSpeed, 52.25f, 13, 90, 1, null);
        float speedMax = signalResult == signalResult.THREE ? 10 : 20;
        float speedMin = signalResult == signalResult.THREE ? 4 : 6;
        float targetX = signalResult == signalResult.THREE ? 38 : 36;
        driveToPosition(speedMax, speedMin, targetX, 13, 90, 4, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90, true, true));
        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.closeClaw();

        if (signalResult == SignalResult.ONE) {
            driveToPosition(new MotionProfile(6, 20, 16), 36f, 59.5f,
                    90, 1, new WiggleProfile(5, 0.1f, 2),
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90, true, true,
                            (d) -> d > 3 && d < 48 && Math.abs(d + 6 - bot.getPose().x) < 10,
                            (d) -> d > 3 && d < 60));
        } else if (signalResult == SignalResult.TWO) {
            driveToPosition(new MotionProfile(6, 20, 16), 36f, 36, 90, 1,
                    new WiggleProfile(5, 0.1f, 2),
                    new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90, true, true));
        }

//        switch(signalResult){
//            case THREE:
//                driveToPosition(highSpeed, 58.5f, 15, -135, 1, null);
//                turnToHeading(90, 3, 6, 150);
//                driveToPosition(highSpeed,  52.75f, 19.25f, 90, 1,
//                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true ));
//                bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
//                sleep(200);
//                bot.openClaw();
//                sleep(200);
//                bot.setClawPosition(OmniBot.LIFT_LOW);
//                sleep(200);
//                driveToPosition(highSpeed, 51.75f, 12, 90, 1,
//                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true ) );
//                break;
//            case TWO:
//                driveToPosition(highSpeed, 57.5f, 13, -135, 1, null);
//                turnToHeading(90, 3, 6, 150);
////                driveToPosition(ultraHighSpeed, 36, 13, 90, 1,
////                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, true, true, (d) -> d>3&&d<40, (d) -> d>3&&d<40));
//                driveToPosition(ultraHighSpeed, 57.5f, 36, 90, 1,
//                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true));
//                break;
//            case ONE:
//                driveToPosition(highSpeed, 57.5f, 13, -135, 1, null);
//                turnToHeading(90, 3, 6, 150);
////                driveToPosition(ultraHighSpeed, 36, 13, 90, 1,
////                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, true, true, (d) -> d>3&&d<40, (d) -> d>3&&d<40));
//                driveToPosition(ultraHighSpeed, 57.5f, 58, 90, 1,
//                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true,
//                                (d) -> d>3 && d < 40 && Math.abs(d+6-bot.getPose().x) < 10, (d) -> d>3&&d<60));
//
//        }

        while (opModeIsActive()) {
            continue;
        }
    }

}

        /*
         * Drive to the appropriate parking location.
         */
        /*driveToPosition(highSpeed, 52.25f, 13, 90, 1, null);
        float speedMax = signalResult == signalResult.THREE? 10:20;
        float speedMin = signalResult == signalResult.THREE? 4:6;
        float targetX = signalResult == signalResult.THREE? 38:36;
        driveToPosition(speedMax,speedMin,targetX,13,90,4,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_90, true, true));

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
*/