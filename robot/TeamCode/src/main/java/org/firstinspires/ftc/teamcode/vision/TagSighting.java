package org.firstinspires.ftc.teamcode.vision;

/** Un tag visto en una foto: qué lugar ocupa en la fila y en qué ángulos apareció. */
public class TagSighting {
    public final int memberIndex;   // 0 a 3
    public final double txDeg;      // + = a la derecha de la mira
    public final double tyDeg;      // + = arriba de la mira
    public final double area;       // % de la imagen
    public final double distanceIn; // distancia horizontal por pose 3D, NaN si no hay

    public TagSighting(int memberIndex, double txDeg, double tyDeg, double area, double distanceIn) {
        this.memberIndex = memberIndex;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.area = area;
        this.distanceIn = distanceIn;
    }
}
