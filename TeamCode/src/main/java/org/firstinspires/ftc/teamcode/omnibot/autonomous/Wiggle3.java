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

@Autonomous(name = "Wiggle3")
public class Wiggle3 extends OmniBotAuto {
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

        // Raise lift to the position for low junction
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(100);

        /*
         * Drive to middle of starting square, then to low junction, then deliver preload cone.
         *
         * TODO: Consider making these drives a little slower (for reliability), and consider
         *  adding Kalman filter (X only) to the first drive.
         */
        driveToPosition(highSpeed, 12, 36, -90, 1, null); // was vmax = 10, vmin = 4
        turnToHeading(0, 3, 6, 150);
        driveToPosition(highSpeed, 17.5f, 41f, 0, 1,
                new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_0, true, false, (d) -> d>3 && d<40, null));

        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        // Drive back to middle of starting square
        driveToPosition(ultraHighSpeed, 12, 35.5f, 0, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive forward, pushing signal out of the way, using Kalman for X only
        driveToPosition(ultraHighSpeed, 36, 35.5f, 0,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        /* Back away, to middle of square, away from signal sleeve, Kalman for X only
         *
         * TODO: Make sure this always drives back far enough to avoid hitting mid junction on
         *  the next drive. If not, may need to tweak. e.g., could change targetX to
         *  bot.getPose().x-3, and possibly remove the Kalman filter to use odometry only.
         */
        driveToPosition(ultraHighSpeed, 34, 35.5f, 0, 1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, true, false));

        // Drive toward right wall, using Kalman for X and Y
        driveToPosition(ultraHighSpeed, 35,13,0,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));


        // Drive in x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( 0, new VectorF(36,13), 0,
                highSpeed, 2, 22.5f,
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
        driveToPosition(highSpeed, 57.5f, 16.5f, 0, 1, null);
        turnToHeading(-135, 3, 6, 150);
        bot.openClaw();
        bot.setLiftPosition(-440);
        sleep(200);
        driveToPosition(highSpeed, 57.5f, 8.5f, -135, 1, null);
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
        driveToPosition(highSpeed, 58.5f, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 120);
        driveToPosition(highSpeed,  51.75f, 18.25f, 90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW + 200);
        sleep(200);
        bot.openClaw();
        sleep(200);
        bot.setClawPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to the colored tape, turn so that grabber is facing cone stack, then
         * adjust position to edge of tape again
         */

        adjustPositionColor(0.5f, 90, 4, 5, 0.1f);
        turnToHeading(-135, 3, 6, 120);
        adjustPositionColor(0.5f, -135, 4, 5, 0.1f);
        bot.setPose(57.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /*
        * Pick up second cone from stack
         */

        bot.openClaw();
        bot.setLiftPosition(-300);
        sleep(300);
        driveToPosition(highSpeed, 57.5f, 8.5f, -135, 1, null);
        bot.closeClaw();
        sleep(300);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Drive back to low junction and drop off cone.
         *
         * TODO: Consider making these drives slower, for reliability.
         */


        driveToPosition(highSpeed, 58.5f, 15, -135, 1, null);
        turnToHeading(90, 3, 6, 150);
        driveToPosition(highSpeed,  52.75f, 19.25f, 90, 1,
                new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true ));
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

        switch(signalResult){
            case THREE:
                driveToPosition(highSpeed, bot.getPose().x, bot.getPose().y-3, 90, 1,null);
                break;
            case TWO:
                driveToPosition(highSpeed, bot.getPose().x+6, bot.getPose().y, 90, 1, null);
                bot.setPose(57.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
                bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                        1, 0, 0, bot.getCovariance().get(1,1)}));
                driveToPosition(ultraHighSpeed, bot.getPose().x, 36, 90, 1,
                        new WiggleProfile(3, 0.1f, 2),
                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true));
                break;
            case ONE:
                driveToPosition(highSpeed, bot.getPose().x+6, bot.getPose().y, 90, 1, null);
                bot.setPose(57.5f, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
                bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                        1, 0, 0, bot.getCovariance().get(1,1)}));
                driveToPosition(ultraHighSpeed, bot.getPose().x, 58, 90, 1,
                        new WiggleProfile(3, 0.1f, 2),
                        new PowerPlayDistUpdater(Quadrant.BLUE_RIGHT, HeadingIndex.H_90, false, true));
                break;
        }

    }

}
