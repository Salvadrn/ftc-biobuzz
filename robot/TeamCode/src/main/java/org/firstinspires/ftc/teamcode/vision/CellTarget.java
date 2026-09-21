package org.firstinspires.ftc.teamcode.vision;

/** Resultado de una foto: hacia dónde está el centro del CELL y a qué distancia. */
public class CellTarget {
    public final Cell cell;
    public final double txDeg;        // ángulo horizontal al CENTRO del CELL, + = derecha
    public final double tyDeg;        // ángulo vertical a la fila de tags, + = arriba
    public final double distanceIn;   // distancia horizontal al CELL, NaN si no se pudo calcular
    public final int tagCount;        // cuántos de los 4 tags se vieron
    public final boolean centered;    // false = se vio un solo tag y no se pudo corregir al centro
    public final long captureTimeNs;  // instante de la foto, en la base de System.nanoTime()

    public CellTarget(Cell cell, double txDeg, double tyDeg, double distanceIn,
                      int tagCount, boolean centered, long captureTimeNs) {
        this.cell = cell;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.distanceIn = distanceIn;
        this.tagCount = tagCount;
        this.centered = centered;
        this.captureTimeNs = captureTimeNs;
    }
}
