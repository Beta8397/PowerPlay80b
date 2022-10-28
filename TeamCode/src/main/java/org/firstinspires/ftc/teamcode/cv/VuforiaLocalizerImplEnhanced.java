package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * Created by FTC Team 8397 on 3/5/2019.
 */
public class VuforiaLocalizerImplEnhanced extends VuforiaLocalizerImpl {

    public VuforiaLocalizerImplEnhanced(Parameters parameters){
        super(parameters);
    }

    @Override
    public void close(){
        super.close();
    }
}
