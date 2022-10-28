package org.firstinspires.ftc.teamcode.util;
/**
 * Represents the position (x,y) and heading (in radians) of the robot on the field.
 * NOTE:  Pose is immutable.
 */
public class MotionProfile {
    public final float vMin;
    public final float vMax;
    public final float accel;

    public MotionProfile(float vMin, float vMax, float accel){
        this.vMin = vMin;
        this.vMax = vMax;
        this.accel = accel;
    }
}
