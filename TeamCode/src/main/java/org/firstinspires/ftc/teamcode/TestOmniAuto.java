package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.omnibot.OmniBotAuto;
@Autonomous(name = "TestOmniAuto")
public class TestOmniAuto extends OmniBotAuto {
    OmniBot bot = new OmniBot();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        super.setBot(bot);
        bot.setPose(0, 0, 90);
        waitForStart();
        driveToPosition(20, 8, 24, 0, 90, 4, 1);
        sleep(1000);
        turnToHeading(0, 3, 3, 45);
    }
}
