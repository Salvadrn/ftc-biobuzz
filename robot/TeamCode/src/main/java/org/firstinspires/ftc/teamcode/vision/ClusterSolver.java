package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

/**
 * Convierte los tags sueltos que ve el Limelight en la dirección al CENTRO de la fila.
 *
 * El problema: el Limelight no conoce los "clusters" del SDK, solo ve tags sueltos.
 * Si apuntamos al promedio de los tags visibles y dos están tapados, el promedio se
 * corre hacia un lado unas 4.6 in.
 *
 * La solución: sabemos en qué lugar de la fila va cada tag (Cell.TAG_X_IN). Con dos o
 * más tags se ajusta una recta "posición en la imagen contra posición en la fila" y se
 * lee en x = 0, el centro de la fila. El ajuste se hace en tan(ángulo), que es donde una
 * recta del mundo sigue siendo recta en la imagen.
 *
 * Con un solo tag se reutiliza la pendiente de la última foto buena, que cambia
 * despacio. Si no hay una reciente, se apunta al tag tal cual.
 *
 * Java puro, sin nada del SDK, para poder probarla en la computadora.
 */
public class ClusterSolver {

    public static class Solution {
        public final double txDeg;
        public final double tyDeg;
        public final boolean centered;
        Solution(double txDeg, double tyDeg, boolean centered) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.centered = centered;
        }
    }

    private final double slopeMemorySeconds;

    // Pendientes de la última foto con 2+ tags, en tan(ángulo) por pulgada de fila
    private double xSlope, ySlope;
    private long slopeTimeNs;
    private Cell slopeCell;

    public ClusterSolver(double slopeMemorySeconds) {
        this.slopeMemorySeconds = slopeMemorySeconds;
    }

    public Solution solve(Cell cell, List<TagSighting> tags, long nowNs) {
        if (tags.isEmpty()) return null;

        if (tags.size() >= 2) {
            double[] fitX = fitLine(tags, true);
            double[] fitY = fitLine(tags, false);
            xSlope = fitX[1];
            ySlope = fitY[1];
            slopeTimeNs = nowNs;
            slopeCell = cell;
            return new Solution(atanDeg(fitX[0]), atanDeg(fitY[0]), true);
        }

        TagSighting only = tags.get(0);
        boolean slopeIsFresh = slopeCell == cell
                && (nowNs - slopeTimeNs) / 1e9 <= slopeMemorySeconds;
        if (slopeIsFresh) {
            double x = Cell.TAG_X_IN[only.memberIndex];
            return new Solution(atanDeg(tanDeg(only.txDeg) - xSlope * x),
                    atanDeg(tanDeg(only.tyDeg) - ySlope * x), true);
        }
        return new Solution(only.txDeg, only.tyDeg, false);
    }

    /** Olvida la geometría guardada (por ejemplo, al cambiar de CELL). */
    public void reset() {
        slopeCell = null;
    }

    /**
     * Mínimos cuadrados de tan(ángulo) = a + b·x sobre los tags vistos.
     * @return {a, b}: a en el centro de la fila, b por pulgada
     */
    private static double[] fitLine(List<TagSighting> tags, boolean horizontal) {
        double n = tags.size();
        double sumX = 0, sumA = 0, sumXX = 0, sumXA = 0;
        for (TagSighting t : tags) {
            double x = Cell.TAG_X_IN[t.memberIndex];
            double a = tanDeg(horizontal ? t.txDeg : t.tyDeg);
            sumX += x;
            sumA += a;
            sumXX += x * x;
            sumXA += x * a;
        }
        double denom = n * sumXX - sumX * sumX;
        if (Math.abs(denom) < 1e-9) {
            // Solo pasa si el mismo tag llega repetido: sin recta posible, se promedia
            return new double[]{sumA / n, 0};
        }
        double b = (n * sumXA - sumX * sumA) / denom;
        double a = (sumA - b * sumX) / n;
        return new double[]{a, b};
    }

    private static double tanDeg(double deg) {
        return Math.tan(Math.toRadians(deg));
    }

    private static double atanDeg(double t) {
        return Math.toDegrees(Math.atan(t));
    }
}
