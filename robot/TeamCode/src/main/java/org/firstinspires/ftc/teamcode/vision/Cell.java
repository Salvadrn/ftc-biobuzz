package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Los cuatro CELLS del campo BIOBUZZ y sus AprilTags.
 *
 * Cada CELL lleva debajo una calcomanía con 4 tags 36h11 de 3.25 in, en fila.
 * Los IDs son consecutivos a partir del primero. Fuente: manual de competencia
 * sección 9.9 y AprilTagGameDatabase.getBioBuzzTagLibrary() del SDK 12.0.
 */
public enum Cell {
    RED_SCORING(30, Alliance.RED),     // lado opuesto a la audiencia
    RED_AUDIENCE(34, Alliance.RED),
    BLUE_AUDIENCE(38, Alliance.BLUE),
    BLUE_SCORING(42, Alliance.BLUE);   // lado opuesto a la audiencia

    public static final int TAGS_PER_CELL = 4;

    /**
     * Posición lateral de cada tag dentro de la fila, en pulgadas, medida desde el
     * centro de la abertura del CELL. La fila es simétrica, así que su centro
     * coincide con el centro de la abertura: a ese punto hay que apuntar.
     */
    public static final double[] TAG_X_IN = {-6.5, -2.75, 2.75, 6.5};

    public final int firstTagId;
    public final Alliance alliance;

    Cell(int firstTagId, Alliance alliance) {
        this.firstTagId = firstTagId;
        this.alliance = alliance;
    }

    /** CELL al que pertenece un tag, o null si el ID no es de BIOBUZZ. */
    public static Cell fromTagId(int tagId) {
        for (Cell c : values()) {
            if (tagId >= c.firstTagId && tagId < c.firstTagId + TAGS_PER_CELL) return c;
        }
        return null;
    }

    /** Lugar del tag dentro de la fila: 0 a 3. */
    public int memberIndex(int tagId) {
        return tagId - firstTagId;
    }
}
