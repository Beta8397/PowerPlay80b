package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    DcMotorEx motor = null;
    boolean reversed = false;

    public Encoder (DcMotorEx motor, boolean reversed) {
        this.motor = motor;
        this.reversed = reversed;
    }

    public int getCurrentPosition() {
        return reversed? -motor.getCurrentPosition(): motor.getCurrentPosition();
    }
}
