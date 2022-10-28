package org.firstinspires.ftc.teamcode.i2c;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;

/**
 * Created by FTC Team 8397 on 9/11/2019.
 */
@I2cDeviceType
@DeviceProperties(xmlTag = "BN0055EnhancedImpl", name = "BN0055EnhancedImpl", description = "BNO055EnhancedImpl")
public class BNO055EnhancedImpl extends BNO055IMUImpl implements BNO055Enhanced {


    @Override
    public String getDeviceName() {
        return "BNO055";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    public BNO055EnhancedImpl(I2cDeviceSynch deviceClient){
        super(deviceClient);
    }
    /**
     * Do one attempt at initializing the device to be running in the indicated operation mode
     */
    protected boolean internalInitializeOnce(SystemStatus expectedStatus)
    {
        // Validate parameters
        if (SensorMode.CONFIG == parameters.mode)
            throw new IllegalArgumentException("SensorMode.CONFIG illegal for use as initialization mode");

        ElapsedTime elapsed = new ElapsedTime();
        if (parameters.accelerationIntegrationAlgorithm != null)
        {
            this.accelerationAlgorithm = parameters.accelerationIntegrationAlgorithm;
        }

        // Make sure we have the right device
        if (!imuIsPresent(deviceClient, true))
        {
            log_e("IMU appears to not be present");
            return false;
        }

        // Get us into config mode, for sure
        setSensorMode(SensorMode.CONFIG);

        // Reset the system, and wait for the chip id register to switch back from its reset state
        // to the chip id state. This can take a very long time, some 650ms (Table 0-2, p13)
        // perhaps. While in the reset state the chip id (and other registers) reads as 0xFF.
        I2cWarningManager.suppressNewProblemDeviceWarnings(true);
        try {
            elapsed.reset();
            write8(Register.SYS_TRIGGER, 0x20, I2cWaitControl.WRITTEN);
            delay(400);
            RobotLog.vv("IMU", "Now polling until IMU comes out of reset. It is normal to see I2C failures below");
            byte chipId;
            while (!Thread.currentThread().isInterrupted())
            {
                chipId = read8(Register.CHIP_ID);
                if (chipId == bCHIP_ID_VALUE)
                    break;
                delayExtra(10);
                if (elapsed.milliseconds() > msAwaitChipId)
                {
                    log_e("failed to retrieve chip id");
                    return false;
                }
            }
            delayLoreExtra(50);
        }
        finally
        {
            I2cWarningManager.suppressNewProblemDeviceWarnings(false);
        }

        RobotLog.vv("IMU", "IMU has come out of reset. No more I2C failures should occur.");

        // Set to normal power mode
        write8(Register.PWR_MODE, POWER_MODE.NORMAL.getValue(), I2cWaitControl.WRITTEN);
        delayLoreExtra(10);

        // Make sure we're looking at register page zero, as the other registers
        // we need to set here are on that page.
        write8(Register.PAGE_ID, 0);

        // Set the output units. Section 3.6, p31
        int unitsel = (parameters.pitchMode.bVal << 7) |       // pitch angle convention
                (parameters.temperatureUnit.bVal << 4) | // temperature
                (parameters.angleUnit.bVal << 2) |       // euler angle units
                (parameters.angleUnit.bVal << 1) |       // gyro units, per second
                (parameters.accelUnit.bVal /*<< 0*/);    // accelerometer units
        write8(Register.UNIT_SEL, unitsel);

        write8(Register.AXIS_MAP_CONFIG, ((BNO055Enhanced.Parameters)parameters).axesMap.bVal);
        write8(Register.AXIS_MAP_SIGN, ((BNO055Enhanced.Parameters)parameters).axesSign.bVal);

        // Switch to page 1 so we can write some more registers
        write8(Register.PAGE_ID, 1);

        // Configure selected page 1 registers
        write8(Register.ACC_CONFIG, parameters.accelPowerMode.bVal | parameters.accelBandwidth.bVal | parameters.accelRange.bVal);
        write8(Register.MAG_CONFIG, parameters.magPowerMode.bVal | parameters.magOpMode.bVal | parameters.magRate.bVal);
        write8(Register.GYR_CONFIG_0, parameters.gyroBandwidth.bVal | parameters.gyroRange.bVal);
        write8(Register.GYR_CONFIG_1, parameters.gyroPowerMode.bVal);

        // Switch back
        write8(Register.PAGE_ID, 0);

        write8(Register.SYS_TRIGGER, 0x00);

        if (this.parameters.calibrationData != null)
        {
            writeCalibrationData(this.parameters.calibrationData);
        }
        else if (this.parameters.calibrationDataFile != null)
        {
            try {
                File file = AppUtil.getInstance().getSettingsFile(this.parameters.calibrationDataFile);
                String serialized = ReadWriteFile.readFileOrThrow(file);
                CalibrationData data = CalibrationData.deserialize(serialized);
                writeCalibrationData(data);
            }
            catch (IOException e)
            {
                // Ignore the absence of the indicated file, etc
            }
        }

        // Finally, enter the requested operating mode (see section 3.3).
        setSensorMode(parameters.mode);

        // Make sure the status is correct before exiting
        SystemStatus status = getSystemStatus();
        if (status==expectedStatus)
            return true;
        else
        {
            log_w("IMU initialization failed: system status=%s expected=%s", status, expectedStatus);
            return false;
        }
    }


    enum POWER_MODE
    {
        NORMAL(0X00),
        LOWPOWER(0X01),
        SUSPEND(0X02);
        //------------------------------------------------------------------------------------------
        protected byte value;
        POWER_MODE(int value) { this.value = (byte)value; }
        public byte getValue() { return this.value; }
    }




}
