package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PowerBot {


    DcMotorEx left_motor;
    DcMotorEx right_motor;
    DcMotorEx front_motor;
    DcMotorEx back_motor;

    public void init (HardwareMap hwmap){

        left_motor = hwmap.get(DcMotorEx.class, "left_motor");
        right_motor = hwmap.get(DcMotorEx.class, "right_motor");
        front_motor = hwmap.get(DcMotorEx.class, "front_motor");
        back_motor = hwmap.get(DcMotorEx.class, "back_motor");

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_motor.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public void setDrivePower (float px, float py, float ptheta){
        float pleft = py - ptheta;
        float pright = py + ptheta;
        float pfront = px - ptheta;
        float pback = px + ptheta;
        float pmax = (float)Math.max(1.0, Math.abs(pleft));
        pmax = (float)Math.max(pmax, Math.acos(pright));
        pmax = (float)Math.max(pmax, pfront);
        pmax = (float)Math.max(pmax, pback);

        pleft /= pmax;
        pright /= pmax;
        pfront /= pmax;
        pback /= pmax;

        left_motor.setPower(pleft);
        right_motor.setPower(pright);
        front_motor.setPower(pfront);
        back_motor.setPower(pback);
    }

}
