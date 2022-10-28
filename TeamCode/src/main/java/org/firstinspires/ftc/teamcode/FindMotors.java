package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name="find motors")
public class FindMotors extends LinearOpMode {
    DcMotorEx p0,p1,p2,p3;

    public void runOpMode(){
        p0=hardwareMap.get(DcMotorEx.class,"p0");
        p1=hardwareMap.get(DcMotorEx.class,"p1");
        p2=hardwareMap.get(DcMotorEx.class,"p2");
        p3=hardwareMap.get(DcMotorEx.class,"p3");


        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                p0.setPower(0.2);
            } else {
                p0.setPower(0);
            }
            if(gamepad1.b){
                p1.setPower(0.2);
            } else {
                p1.setPower(0);
            }
            if(gamepad1.x){
                p2.setPower(0.2);
            } else {
                p2.setPower(0);
            }
            if(gamepad1.y){
                p3.setPower(0.2);
            } else {
                p3.setPower(0);
            }
        }
    }
}
