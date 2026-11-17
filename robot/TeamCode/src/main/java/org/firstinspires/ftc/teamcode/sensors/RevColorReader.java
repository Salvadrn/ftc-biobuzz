package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Adaptador del REV Color Sensor V3 (REV-31-1557). En la configuración se da de alta
 * como "REV Color Sensor V3". Los 3 del robot tienen la misma dirección I2C (0x52):
 * cada uno va en su propio bus.
 */
public class RevColorReader implements ColorReader {
    private final NormalizedColorSensor color;
    private final DistanceSensor distance;

    public RevColorReader(HardwareMap hardwareMap, String name, float gain) {
        color = hardwareMap.get(NormalizedColorSensor.class, name);
        distance = hardwareMap.get(DistanceSensor.class, name);
        color.setGain(gain);
    }

    @Override
    public boolean read(double[] out) {
        NormalizedRGBA c = color.getNormalizedColors();
        out[0] = c.red;
        out[1] = c.green;
        out[2] = c.blue;
        out[3] = distance.getDistance(DistanceUnit.MM);
        return !Double.isNaN(out[3]);
    }
}
