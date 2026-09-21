package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

/**
 * Convierte los tags sueltos que ve el Limelight en el ángulo al CENTRO del CELL.
 *
 * El problema: el Limelight no conoce los "clusters" del SDK, solo ve tags sueltos.
 * Si apuntamos al promedio de los tags visibles y un tag está tapado, el promedio se
 * corre hacia un lado y el tiro sale chueco.
 *
 * La solución: sabemos en qué lugar de la fila va cada tag (Cell.TAG_X_IN). Con dos o
 * más tags ajustamos una recta "ángulo contra posición en la fila" y leemos el ángulo
 * en x = 0, que es el centro de la abertura. Funciona igual con los tags 0 y 1 que con
 * los cuatro, y no importa desde qué lado se mire la fila: el signo sale del ajuste.
 *
 * Con un solo tag no hay recta posible, así que se reutiliza la pendiente de la última
 * foto buena, que cambia despacio. Si no hay ninguna reciente se apunta al tag tal cual.
 *
 * Esta clase es Java puro, sin nada del SDK, para poder probarla en la computadora.
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

    // Pendientes de la última foto con 2+ tags, en grados por pulgada de fila
    private double txSlope, tySlope;
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
            txSlope = fitX[1];
            tySlope = fitY[1];
            slopeTimeNs = nowNs;
            slopeCell = cell;
            return new Solution(fitX[0], fitY[0], true);
        }

        TagSighting only = tags.get(0);
        boolean slopeIsFresh = slopeCell == cell
                && (nowNs - slopeTimeNs) / 1e9 <= slopeMemorySeconds;
        if (slopeIsFresh) {
            double x = Cell.TAG_X_IN[only.memberIndex];
            return new Solution(only.txDeg - txSlope * x, only.tyDeg - tySlope * x, true);
        }
        return new Solution(only.txDeg, only.tyDeg, false);
    }

    /**
     * Mínimos cuadrados de ángulo = a + b·x sobre los tags vistos.
     * @return {a, b}: a es el ángulo en el centro de la fila, b los grados por pulgada
     */
    private static double[] fitLine(List<TagSighting> tags, boolean horizontal) {
        double n = tags.size();
        double sumX = 0, sumA = 0, sumXX = 0, sumXA = 0;
        for (TagSighting t : tags) {
            double x = Cell.TAG_X_IN[t.memberIndex];
            double angle = horizontal ? t.txDeg : t.tyDeg;
            sumX += x;
            sumA += angle;
            sumXX += x * x;
            sumXA += x * angle;
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
}
