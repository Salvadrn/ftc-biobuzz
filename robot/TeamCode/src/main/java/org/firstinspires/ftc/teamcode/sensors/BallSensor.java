package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.util.ColorClassifier;

/**
 * Un sensor de color que dice qué pelota tiene enfrente.
 *
 * Cada lectura I2C del V3 tarda unos milisegundos y el robot tiene 3. Para no frenar el
 * ciclo, cada sensor se lee como mucho una vez cada minIntervalS y entre lecturas se
 * regresa la última.
 *
 * Si la lectura falla (cable suelto, I2C caído), isFault() es true: quien lo use NO debe
 * tomarlo como "no hay pelota".
 */
public class BallSensor {
    private final ColorReader reader;
    private final ColorClassifier.Thresholds thresholds;
    private final long minIntervalNs;

    private final double[] raw = new double[4];
    private long lastReadNs = Long.MIN_VALUE / 2;
    private BallType last = BallType.NONE;
    private boolean fault = false;

    public BallSensor(ColorReader reader, ColorClassifier.Thresholds thresholds, double minIntervalS) {
        this.reader = reader;
        this.thresholds = thresholds;
        this.minIntervalNs = (long) (minIntervalS * 1e9);
    }

    /** Lee si ya toca; si no, regresa lo último. Con falla regresa UNKNOWN. */
    public BallType read(long nowNs) {
        if (nowNs - lastReadNs >= minIntervalNs) {
            lastReadNs = nowNs;
            fault = !reader.read(raw);
            last = fault ? BallType.UNKNOWN : ColorClassifier.classify(raw[0], raw[1], raw[2], raw[3], thresholds);
        }
        return last;
    }

    public BallType last() { return last; }
    public boolean hasBall() { return last.isBall(); }
    public boolean isFault() { return fault; }

    /** Algún canal saturado: el color no es confiable (bajar COLOR_GAIN). */
    public boolean isSaturated() {
        return raw[0] >= 0.99 || raw[1] >= 0.99 || raw[2] >= 0.99;
    }

    /** Para telemetría y calibración: {r, g, b, mm} de la última lectura. */
    public double[] raw() { return raw.clone(); }

    public String describe() {
        if (fault) return "SIN LECTURA (¿cable o I2C?)";
        double[] hsv = ColorClassifier.rgbToHsv(raw[0], raw[1], raw[2]);
        return String.format("%s  %.0f mm  hue %.0f  sat %.2f%s", last, raw[3], hsv[0], hsv[1],
                isSaturated() ? "  SATURADO" : "");
    }
}
