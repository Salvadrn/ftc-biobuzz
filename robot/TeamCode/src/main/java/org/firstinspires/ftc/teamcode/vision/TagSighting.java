package org.firstinspires.ftc.teamcode.vision;

/** Un tag visto en una foto: qué lugar ocupa en la fila y dónde apareció. */
public class TagSighting {
    public final int memberIndex;    // 0 a 3
    public final double txDeg;       // + = derecha, sin crosshair
    public final double tyDeg;       // + = arriba, sin crosshair
    public final double areaPct;     // área del tag en % de la imagen ("ta")
    public final double height3dIn;  // altura del tag por la pose 3D, NaN si no hay

    public TagSighting(int memberIndex, double txDeg, double tyDeg, double areaPct, double height3dIn) {
        this.memberIndex = memberIndex;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.areaPct = areaPct;
        this.height3dIn = height3dIn;
    }
}
