package org.firstinspires.ftc.teamcode.i2c;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.nio.ByteOrder;
import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(xmlTag = "RevColorV2", name = "RevColorV2", description = "RevColorV2")
public class RevColorV2 extends LynxI2cColorRangeSensor {

    public RevColorV2(I2cDeviceSynchSimple deviceClient){
        super(deviceClient);
    }

    public void setHardwareGain(AMSColorSensor.Gain gain){
        super.setHardwareGain(gain);
    }

    public RawColors getRawColors(){
        {
            // Wait for data to be valid. But don't wait forever: it's basically never
            // a good idea to wait forever. Be efficient and only use one I2c transaction
            // in the case where the data is already valid, which is the common case.
            //
            Deadline deadline = new Deadline(2, TimeUnit.SECONDS);
            byte[] data = null;
            for (;;)
            {
                // Read STATUS, ALPHA, RED, GREEN & BLUE
                final int cbRead = Register.PDATA.bVal - Register.STATUS.bVal;
                data = read(Register.STATUS, cbRead);

                // Is the data valid? Carry on if so
                if (testBits(data[0], Status.AVALID.bVal)) break;

                // Get out of here if we should; otherwise, briefly wait then try again
                if (Thread.currentThread().isInterrupted() || !isConnectedAndEnabled() || deadline.hasExpired())
                {
                    // Return fake data and get out of here
                    return new RawColors(0,0,0,0);
                }
                delay(3);
            }

            final int dib = 1;
            int alpha = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib + 0, ByteOrder.LITTLE_ENDIAN));
            int red   = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib + 2, ByteOrder.LITTLE_ENDIAN));
            int green = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib + 4, ByteOrder.LITTLE_ENDIAN));
            int blue  = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib + 6, ByteOrder.LITTLE_ENDIAN));

            return new RawColors(alpha, red, green, blue);

        }
    }

}
