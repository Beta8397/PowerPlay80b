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
 * Deliver three to high junction and park (RIGHT SIDE)
 */
@Autonomous(name = "Harrison's Experiment")
public class WiggleHighTest extends OmniBotAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.setPose(8, 36, -90);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcamName);

        bot.closeClaw();
        bot.grabPosition();

        waitForStart();

        signalResult = getSignalResult();
        telemetry.addData("Signal", signalResult);
        telemetry.update();

        // Drive to high junction
        bot.setLiftPosition(-100);
        driveToPosition(highSpeed, 63, 36, -90, 1,null);
        bot.setLiftPosition(OmniBot.LIFT_MID);
        driveToPosition(midSpeed, 60, 36, -90, 1,null);

        bot.setLiftPosition(OmniBot.LIFT_HIGH);
        driveToPosition(highSpeed, 60, 51, -90, 1, null);

        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        sleep(400);
        driveToPosition(midSpeed, 61.5f, 51, -90, 1, null);

        // Drop off cone
        bot.setLiftPosition(OmniBot.LIFT_HIGH + 300);
        sleep(200);
        bot.openClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_HIGH);
        sleep(200);


        driveToPosition(highSpeed, 60, 45, -90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW);

        bot.setPivotPosition(OmniBot.PIVOT_FRONT);

        driveToPosition(ultraHighSpeed, 60,16,-90,1, null);
        bot.openClaw();

        // Drive in -x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( -90, new VectorF(60,16),-180,
                midSpeed, 2, 1, null,
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        float xOffset = -3;
        float adjustedX = X_TAPE_EDGE + xOffset;
        adjustPositionColor(0.5f, xOffset,-90, 4, 5, 0.1f);

        bot.setPose(adjustedX, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab first stack cone
         */
        /*driveToPosition(midSpeed, adjustedX, 16, -90, 1,
                new WiggleProfile(3, 0.1f, 2.0f), null);*/
        bot.openClaw();
        bot.setLiftPosition(-460);
        sleep(200);
        driveToPosition(midSpeed, adjustedX, 11, -90, 1, new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_NEG_90, false, true));
        bot.closeClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Back away from cone stack, drive to junction, and drop off first stack cone on high junction
         */
        driveToPosition(highSpeed, adjustedX, 15, -90, 1, null);
       // driveToPosition(highSpeed,  59, 15, -90, 1,new WiggleProfile(3, 0.1f, 2),null);

        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        bot.setLiftPosition(OmniBot.LIFT_HIGH);

        driveToPosition(ultraHighSpeed,  59, 45, -90, 1, new WiggleProfile(5, 0.1f, 2),null);

        driveToPosition(midSpeed,  61, 51.5f, -90, 1, null);


        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_HIGH + 200);
        sleep(200);
        bot.openClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_HIGH);
        sleep(200);

        driveToPosition(highSpeed,  59, 51.5f, -90, 1, null);

        //go to pick up second cone and score on high junction

        driveToPosition(highSpeed, 60, 36, -90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_LOW);

        bot.setPivotPosition(OmniBot.PIVOT_FRONT);

        driveToPosition(highSpeed, 60,16,-90,1, new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_NEG_90, false, true));
        bot.setClawPosition(OmniBot.PIVOT_FRONT);

        // Drive in -x direction to cone stack. Stop when color sensor detects red or blue tape
        driveLine( -90, new VectorF(60,16),-180,
                highSpeed, 2, 1, null,
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);

        //Adjust x position until color sensor is at near edge of tape
        adjustPositionColor(0.5f, xOffset,-90, 4, 5, 0.1f);

        bot.setPose(adjustedX, bot.getPose().y, (float)Math.toDegrees(bot.getPose().theta));
        bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                1, 0, 0, bot.getCovariance().get(1,1)}));

        /* Back away from the cone stack a few inches, turn so claw faces stack, drive back to stack
         * and grab second stack cone
         */
        driveToPosition(midSpeed, adjustedX, 16, -90, 1, null);
        bot.openClaw();
        bot.setLiftPosition(-380);
        sleep(200);
        driveToPosition(midSpeed, adjustedX, 11, -90, 1, new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_NEG_90, false, true));
        bot.closeClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_LOW);
        sleep(200);

        /*
         * Back away from cone stack, drive to junction, and drop off second stack cone on high junction
         */
        driveToPosition(highSpeed, adjustedX, 15, -90, 1, null);

        driveToPosition(highSpeed,  59, 15, -90, 1, new WiggleProfile(3, 0.1f, 2),null);

        bot.setPivotPosition(OmniBot.PIVOT_SCORING);
        bot.setLiftPosition(OmniBot.LIFT_HIGH);

        driveToPosition(highSpeed,  59, 45, -90, 1, new WiggleProfile(3, 0.3f, 3),null);

        driveToPosition(midSpeed,  61, 51.5f, -90, 1, null);


        sleep(200);
        bot.setLiftPosition(OmniBot.LIFT_HIGH + 300);
        sleep(200);
        bot.openClaw();
        sleep(400);
        bot.setLiftPosition(OmniBot.LIFT_HIGH);
        sleep(200);


        driveToPosition(highSpeed,  59, 51.5f, -90, 1, null);


        driveToPosition(midSpeed,  58, 50.5f, -90, 1, null);
        bot.setLiftPosition(OmniBot.LIFT_MAX);
        bot.setPivotPosition(OmniBot.PIVOT_GRABBING);




        switch(signalResult){
            case THREE:
                driveToPosition(ultraHighSpeed, bot.getPose().x-2, 15, -90, 1,null);
                break;
            case TWO:
                bot.setCovariance(new GeneralMatrixF(2,2, new float[]{
                        1, 0, 0, bot.getCovariance().get(1,1)}));
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

