package org.firstinspires.ftc.teamcode.util;

/**
 * Tabla de interpolación lineal: dada una lista de puntos (x, y) ordenados por x,
 * devuelve la y que corresponde a cualquier x intermedia.
 *
 * Se usa para convertir distancia al objetivo en RPM del flywheel y en posición del hood.
 * Fuera del rango de la tabla NO extrapola: se queda con el valor del extremo.
 */
public class InterpTable {
    private final double[] xs;
    private final double[] ys;

    /** @param points filas {x, y}. Deben venir ordenadas de menor a mayor x. */
    public InterpTable(double[][] points) {
        if (points.length == 0) {
            throw new IllegalArgumentException("La tabla necesita al menos un punto");
        }
        xs = new double[points.length];
        ys = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            xs[i] = points[i][0];
            ys[i] = points[i][1];
            if (i > 0 && xs[i] <= xs[i - 1]) {
                throw new IllegalArgumentException("Las x de la tabla deben ir en orden creciente");
            }
        }
    }

    public double get(double x) {
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];

        int i = 1;
        while (xs[i] < x) i++;
        double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        return ys[i - 1] + t * (ys[i] - ys[i - 1]);
    }

    public double minX() { return xs[0]; }
    public double maxX() { return xs[xs.length - 1]; }
}
