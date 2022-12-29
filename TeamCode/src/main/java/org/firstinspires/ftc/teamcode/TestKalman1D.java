package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous(name = "TestKalman")
public class TestKalman1D extends OmniBotAuto {

    public void runOpMode(){
        bot.init(hardwareMap);

        bot.setPose(36, 36, 0);

        waitForStart();

        // Drive toward right wall, using Kalman for Y
        driveToPosition(16,4,36,12,0,4,1,
                new PowerPlayDistUpdater(Quadrant.RED_RIGHT, HeadingIndex.H_0, false, true));


        // Drive in x direction to cone stack.
        driveLine(0, new VectorF(36,12), 0,
                new MotionProfile(4, 16, 10), 2, 22.5f,
                new KalmanDistanceUpdater(null, null, null,
                        bot.rightDist, (d)->d+6, (d)->d>3 && d< 48 && bot.getPose().x < 50),
                ()->bot.getHSV()[1]>0.4 || bot.getPose().x>64);
    }
}
