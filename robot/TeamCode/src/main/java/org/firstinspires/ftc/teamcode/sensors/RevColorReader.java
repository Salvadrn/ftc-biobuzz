package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Adaptador del REV Color Sensor V3 (REV-31-1557). En la configuración se da de alta
 * como "REV Color Sensor V3". Los 3 del robot tienen la misma dirección I2C (0x52):
 * cada uno va en su propio bus.
 *
 * Lo que hace el SDK 12.0 (leído con javap) y aquí se toma en cuenta:
 *  - De fábrica el sensor mide color y distancia cada 100 ms. Aquí se pone a 25 ms.
 *  - setGain es ganancia de software: multiplica y recorta a 1. Con mucha, se satura.
 *  - Si el I2C falla, el SDK no avisa: regresa bytes en cero (color 0 y 152.4 mm, lo mismo
 *    que "no hay nada"). Por eso se revisa la salud del dispositivo y esa firma exacta.
 */
public class RevColorReader implements ColorReader {
    private final NormalizedColorSensor color;
    private final DistanceSensor distance;
    private final float gain;
    private HardwareDeviceHealth health;

    public RevColorReader(HardwareMap hardwareMap, String name, float gain) {
        this.color = hardwareMap.get(NormalizedColorSensor.class, name);
        this.distance = hardwareMap.get(DistanceSensor.class, name);
        this.gain = gain;
        if (color instanceof RevColorSensorV3) {
            RevColorSensorV3 v3 = (RevColorSensorV3) color;
            BroadcomColorSensor.Parameters p = v3.getParameters().clone();   // clone: el original es interno
            p.proximityMeasRate = BroadcomColorSensor.PSMeasurementRate.R25ms;
            p.lightSensorMeasRate = BroadcomColorSensor.LSMeasurementRate.R25ms;
            v3.initialize(p);
            health = v3.getDeviceClient();
        }
        color.setGain(gain);
    }

    @Override
    public boolean read(double[] out) {
        if (health != null && health.getHealthStatus() == HardwareDeviceHealth.HealthStatus.UNHEALTHY) {
            return false;
        }
        NormalizedRGBA c = color.getNormalizedColors();
        out[0] = c.red;
        out[1] = c.green;
        out[2] = c.blue;
        out[3] = distance.getDistance(DistanceUnit.MM);
        boolean zeroSignature = c.red == 0 && c.green == 0 && c.blue == 0 && out[3] >= 152.0;
        return !zeroSignature && !Double.isNaN(out[3]);
    }

    public float getGain() {
        return gain;
    }
}
