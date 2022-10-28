package org.firstinspires.ftc.teamcode.util;

public class AngleUtil {
    public static float normalizeRadians(float radians){
        if(radians > Math.PI){
            return radians - 2 * (float)Math.PI;
        } else if(radians < -Math.PI){
            return radians + 2 * (float)Math.PI;
        } else return radians;
    }
}
