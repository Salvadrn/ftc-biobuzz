package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.BallType;

/**
 * Convierte la lectura de un REV Color Sensor V3 en un tipo de pelota.
 *
 * Java puro, sin nada del SDK, para poder probarlo en la computadora.
 *
 * Primero decide si hay pelota con la distancia del sensor; luego el color con el
 * matiz (hue). Si el color no alcanza, usa la distancia como pista de tamaño: en la copa
 * la NECTAR queda pegada a la ventana y la POLLEN más lejos (CALC, CAD v131: 4.5 mm contra
 * 16.2 mm). En la garganta no sirve (NECTAR de 8.3 a 14.1 mm y POLLEN de 6.3 a 33.1 según
 * el lado de la fila): ahí nectarMaxMm va en NaN y solo cuenta el color.
 */
public final class ColorClassifier {

    /** Umbrales de un sensor. Cada sensor del robot tiene los suyos en Config. */
    public static final class Thresholds {
        public double presentMaxMm;      // más lejos que esto = no hay pelota
        public double nectarMaxMm;       // más cerca que esto = tamaño de NECTAR (NaN = no usar)
        public double minSaturation;     // debajo de esto el color es gris: no se confía
        public double minValue;          // debajo de esto está muy oscuro

        public Thresholds(double presentMaxMm, double nectarMaxMm, double minSaturation, double minValue) {
            this.presentMaxMm = presentMaxMm;
            this.nectarMaxMm = nectarMaxMm;
            this.minSaturation = minSaturation;
            this.minValue = minValue;
        }
    }

    /** Rangos de matiz en grados (0 a 360). */
    public static double yellowMinHue = 35, yellowMaxHue = 85;
    public static double blueMinHue = 185, blueMaxHue = 260;
    public static double redMaxHue = 25, redMinHueWrap = 325;

    private ColorClassifier() {}

    /**
     * @param r, g, b    colores normalizados del sensor (0 a 1 aprox.)
     * @param distanceMm distancia que reporta el sensor, en mm (NaN si no se sabe)
     */
    public static BallType classify(double r, double g, double b, double distanceMm, Thresholds t) {
        if (Double.isNaN(distanceMm) || distanceMm > t.presentMaxMm) return BallType.NONE;

        double[] hsv = rgbToHsv(r, g, b);
        double hue = hsv[0], sat = hsv[1], val = hsv[2];

        boolean colorUsable = sat >= t.minSaturation && val >= t.minValue;
        if (colorUsable) {
            if (hue >= yellowMinHue && hue <= yellowMaxHue) return BallType.POLLEN;
            if (hue >= blueMinHue && hue <= blueMaxHue) return BallType.NECTAR_BLUE;
            if (hue <= redMaxHue || hue >= redMinHueWrap) return BallType.NECTAR_RED;
        }

        // Sin color confiable: el tamaño solo dice POLLEN o "grande de color desconocido"
        if (!Double.isNaN(t.nectarMaxMm)) {
            return distanceMm <= t.nectarMaxMm ? BallType.UNKNOWN : BallType.POLLEN;
        }
        return BallType.UNKNOWN;
    }

    /** @return {hue 0..360, saturación 0..1, valor 0..max(r,g,b)} */
    public static double[] rgbToHsv(double r, double g, double b) {
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double delta = max - min;

        double hue;
        if (delta <= 1e-12) {
            hue = 0;
        } else if (max == r) {
            hue = 60 * (((g - b) / delta) % 6);
        } else if (max == g) {
            hue = 60 * ((b - r) / delta + 2);
        } else {
            hue = 60 * ((r - g) / delta + 4);
        }
        if (hue < 0) hue += 360;

        double sat = max <= 1e-12 ? 0 : delta / max;
        return new double[]{hue, sat, max};
    }
}
