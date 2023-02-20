package org.firstinspires.ftc.teamcode.i2c;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        name = "Rev2mDistSensorExEx",
        description = "Rev2mDistSensorExEx",
        xmlTag = "Rev2mDistSensorExEx"
)
public class Rev2mDistSensorExEx extends Rev2mDistanceSensorEx {

    Function<Double,Double> calibrationFunctionInch = null;

    public Rev2mDistSensorExEx(I2cDeviceSynch deviceClient){
        super(deviceClient);
    }

    @Override
    public double getDistance(DistanceUnit unit){
        double inches = super.getDistance(DistanceUnit.INCH);
        if (calibrationFunctionInch != null){
            inches = calibrationFunctionInch.apply(inches);
        }
        return unit.fromInches(inches);
    }

    public void setCalibrationFunctionInch(Function calFunction){
        calibrationFunctionInch = calFunction;
    }

}
