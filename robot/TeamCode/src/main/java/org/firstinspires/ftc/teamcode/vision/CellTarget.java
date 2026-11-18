package org.firstinspires.ftc.teamcode.vision;

/** Resultado de una foto: dónde está la abertura de la CELL de arriba vista desde el eje de la torreta. */
public class CellTarget {
    public final Cell cell;
    public final double bearingDeg;    // rumbo desde el eje de la torreta, + = izquierda
    public final double distanceIn;    // distancia horizontal del eje de la torreta al punto de tiro
    public final int tagCount;         // cuántos de los 4 tags se vieron
    public final boolean centered;     // false = un solo tag sin corrección al centro de la fila
    /**
     * true = 2 o más tags: se apuntó al centro de la ABERTURA (3.41 in afuera de la fila).
     * Solo con estas fotos la torreta puede decir "fijo".
     */
    public final boolean precise;
    public final long captureTimeNs;   // instante de la foto, en la base de System.nanoTime()

    public CellTarget(Cell cell, double bearingDeg, double distanceIn, int tagCount,
                      boolean centered, boolean precise, long captureTimeNs) {
        this.cell = cell;
        this.bearingDeg = bearingDeg;
        this.distanceIn = distanceIn;
        this.tagCount = tagCount;
        this.centered = centered;
        this.precise = precise;
        this.captureTimeNs = captureTimeNs;
    }
}
