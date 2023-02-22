package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.omnibot.OmniBot;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "CalibrateDist")
public class CalibrateDist extends LinearOpMode {

    OmniBot bot = new OmniBot();

    DistanceSensor sensor = null;
    String sensorString = null;

    final float[] distances = {6, 18, 30, 42};
    float[] avgDistances = new float[distances.length];
    float[] stdDevs = new float[distances.length];
    float[] sumXPowers = new float[5];
    float[] sumYXPowers = new float[3];
    final int NUM_SAMPLES = 25;

    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    public void runOpMode(){
        bot.init(hardwareMap);

        telemetry.addData("Press START when ready","");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Use DPAD to select a sensor.","");
            telemetry.update();
            if (gamepad1.dpad_up){
                sensor = bot.frontDist;
                sensorString = "FRONT";
                break;
            } else if (gamepad1.dpad_down){
                sensor = bot.backDist;
                sensorString = "BACK";
                break;
            } else if (gamepad1.dpad_left){
                sensor = bot.leftDist;
                sensorString = "LEFT";
                break;
            } else if (gamepad1.dpad_right){
                sensor = bot.rightDist;
                sensorString = "RIGHT";
                break;
            }
        }

        for (int i=0; i<distances.length; i++){
            if (!opModeIsActive()) return;
            toggleA.update();
            while(opModeIsActive()){
                telemetry.addData("","Position %s at %.0f Inches; press A when ready",
                        sensorString, distances[i]);
                telemetry.update();
                if (toggleA.update()) break;
            }
            sampleDistance(i);
        }

        VectorF bLin = new VectorF(sumYXPowers[0], sumYXPowers[1]);
        VectorF bQuad = new VectorF(sumYXPowers[0], sumYXPowers[1], sumYXPowers[2]);

        MatrixF MLin = new GeneralMatrixF(2,2);
        for (int row=0; row<2; row++)
            for (int col=0; col<2; col++) MLin.put(row, col, sumXPowers[row+col]);

        MatrixF MQuad = new GeneralMatrixF(3,3);
        for (int row=0; row<3; row++)
            for (int col=0; col<3; col++) MQuad.put(row, col, sumXPowers[row+col]);

        VectorF aLin = (MLin.inverted()).multiplied(bLin);
        VectorF aQuad = (MQuad.inverted()).multiplied(bQuad);



        telemetry.addData("FINISHED",sensorString);
        telemetry.addData("Avg Readings", stringFromArray(avgDistances));
        telemetry.addData("Std Devs", stringFromArray(stdDevs));
        telemetry.addData("Lin Regression", stringFromArray(aLin.getData()));
        telemetry.addData("Quad Regression", stringFromArray2(aQuad.getData()));
        telemetry.update();

        while (opModeIsActive()) continue;

    }

    private void sampleDistance(int i){
        float prevDist = -1;
        float sum = 0;
        float sumSqr = 0;

        //Sample NUM_SAMPLES unique distances, maintaining sum of distances and sum of squares
        for (int j=0; j<NUM_SAMPLES; j++){
            if (!opModeIsActive()) return;
            while (opModeIsActive()){
                float dist = (float)sensor.getDistance(DistanceUnit.INCH);
                if (dist != prevDist){
                    prevDist = dist;
                    sum += dist;
                    sumSqr += dist*dist;
                    break;
                }
            }
        }

        //Calculate avg and std dev of samples, and save in avgDistances and stdDevs arrays
        float avg = sum/NUM_SAMPLES;
        float stdDev = (float)Math.sqrt(sumSqr/NUM_SAMPLES - avg*avg);
        avgDistances[i] = avg;
        stdDevs[i] = stdDev;

        //Add appropriate values to the sumXPowers and sumYXPowers arrays
        for (int j=0; j<5; j++) sumXPowers[j] = sumXPowers[j] + (float)Math.pow(avg, j);
        for (int j=0; j<3; j++) sumYXPowers[j] = sumYXPowers[j] + distances[i]*(float)Math.pow(avg,j);
    }

    private String stringFromArray(float[] array){
        StringBuilder sb = new StringBuilder();
        for (float f: array){
            String s = String.format("%.3f ", f);
            sb.append(s);
        }
        return sb.toString();
    }

    private String stringFromArray2(float[] array){
        StringBuilder sb = new StringBuilder();
        for (float f: array){
            String s = String.format("%.4f ", f);
            sb.append(s);
        }
        return sb.toString();
    }


}
