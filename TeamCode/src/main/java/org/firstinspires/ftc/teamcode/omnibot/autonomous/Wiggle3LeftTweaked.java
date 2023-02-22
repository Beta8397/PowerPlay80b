package org.firstinspires.ftc.teamcode.omnibot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.WiggleProfile;

/**
 *  Deliver preload cone, then two stacked cones to low junctions. Then park
 *  in the appropriate zone.
 *
 *  This opmode currently works fairly well, but not as reliable as two-cone. It has about
 *  2 or 3 extra seconds available, which could be used to tweak reliability.
 *
 *  TODO: Test thoroughly. Consider adding Wiggle to additional drives. Hard to know which
 *   drives will have skidding on competition field.
 *
 */

@Autonomous(name = "Wiggle3LeftTweaked")
public class Wiggle3LeftTweaked extends OmniBotAuto {
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

        // Raise lift to the position for low junction
        bot.setLiftPosition(OmniBot.LIFT_MID - 100);
        //sleep(100);

        /*
         * Drive to middle of starting square, then to low junction, then deliver preload cone.
         */
        driveToPosition(highSpeed, 39, 105, -90, 1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, true, false)); // was vmax = 10, vmin = 4
        driveToPosition(midSpeed, 36, 105, -90, 1, null);

        driveToPosition(midSpeed, 36, 97, -90, 1, null);

        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        sleep(400);
        driveToPosition(lowSpeed, 38, 97, -90, 1, null);

        bot.setLiftPosition(OmniBot.LIFT_MID + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_MID - 600);
        sleep(200);

        driveToPosition(highSpeed, 36, 105, -90, 1, null);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);
        bot.setLiftPosition(OmniBot.LIFT_LOW);

        /* Back away, to middle of square, away from signal sleeve, Kalman for X only
         *
         * TODO: Make sure this always drives back far enough to avoid hitting mid junction on
         *  the next drive. If not, may need to tweak. e.g., could change targetX to
         *  bot.getPose().x-3, and possibly remove the Kalman filter to use odometry only.
         */


        // Drive toward left wall, using Kalman for X and Y
        driveToPosition(ultraHighSpeed, 35,124,-90,1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_NEG_90, false, true));


        // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( -90, new VectorF(36,124), 0,
                highSpeed, 2, 22.5f,null, ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);
//                new KalmanDistanceUpdater(null, null, null,
//                        bot.backDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
//                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        adjustPositionColor(0.5f,3, -90, 4, 5, 0.1f);

        bot.setPose(60.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab cone
         */
        driveToPosition(highSpeed, 60.5f, 124.5f, -90, 1, null);
        turnToHeading(45, 3, 6, 150);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(200);
        driveToPosition(highSpeed, 60.5f, 130.5f, 45, 1, null);
        bot.closeClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Back away from cone stack, turn, drive to junction, and drop off cone.
         *
         * TODO: Consider making the drives to the junction slower, to improve reliability. Could
         *  add Kalman Filter to the second drive (the one with targetThetaDegrees=90), using
         *  Y only.
         */
        driveToPosition(highSpeed, 60.5f, 123, 45, 1, null);
        turnToHeading(180, 3, 6, 120);
        driveToPosition(highSpeed,  52.25f, 122.25f, 180, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to the colored tape, turn so that grabber is facing cone stack, then
         * adjust position to edge of tape again
         */

        adjustPositionColor(0.5f, 2, 180, 4, 5, 0.1f);

        //adjustPositionColor(0.5f, 3, 45, 4, 5, 0.1f);
        bot.setPose(59.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));
        turnToHeading(45, 3, 6, 120);

        /*
         * Pick up second cone from stack
         */

        bot.openClaw();
        bot.setLiftPosition(-300);
        sleep(300);
        driveToPosition(highSpeed, 59.5f, 130.5f, 45, 1, null);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to low junction and drop off cone.
         *
         * TODO: Consider making these drives slower, for reliability.
         */


        driveToPosition(highSpeed, 59.5f, 123f, 45, 1, null);
        turnToHeading(180, 3, 6, 150);
        driveToPosition(highSpeed,  53.25f, 124.25f, 180, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive to appropriate parking zone.
         *
         * TODO: For case THREE, change parking position a little, so not disturbing the cone
         *  stack. For cases ONE and TWO, consider changing the "bot.getPose().x+6" to something
         *  a little smaller, maybe "bot.getPose().x+4", to avoid running into the far junctions
         *  on the subsequent drives. Also, for cases ONE and TWO, the "bot.setPose()" and
         *  "bot.setCovariance()" statements are not really doing anything. Would try commenting
         *  them out, then getting rid of them if everything still works. Also for cases ONE and
         *  TWO, consider increasing Wiggle duration to 5 seconds.
         *
         */

        driveToPosition(highSpeed, 58.75f, 128.75f,180, 1, null);
        driveToPosition(highSpeed, 35, 128.75f, 180, 1,
                new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_180, true, true));
        switch(signalResult){
            case THREE:
                driveToPosition(highSpeed, 35, 84, 180, 1, new WiggleProfile(5, 0.1f, 2),
                        new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_180, true, true,
                                (d)-> d>3 && d<48 && Math.abs(d + 6 - bot.getPose().x) < 10, (d) -> d>3 && d>48));
                break;
            case TWO:
                driveToPosition(highSpeed, 35, 108, 180, 1, new WiggleProfile(5, 0.1f, 2),
                        new PowerPlayDistUpdater(Quadrant.RED_LEFT, HeadingIndex.H_180, true, true,
                                (d)-> d>3 && d<48 && Math.abs(d + 6 - bot.getPose().x) < 10, (d) -> d>3 && d>48));
                break;
            case ONE:
             break;
        }

    }

}

