package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Cuál de las dos CELLS de nuestra alianza está arriba.
 *
 *  - Arranca con el estado de inicio del partido (manual TU09, fig. 10-2).
 *  - El HIVE solo se voltea cuando le meten pelotas, así que la creencia es pegajosa:
 *    cambia solo con varias fotos SEGUIDAS que lo midan (altura de la fila por separación
 *    de tags o por pose 3D). Si pasa mucho tiempo sin fotos que decidan, la racha se pierde.
 *  - Lo que dice el piloto gana, y solo se deshace con UPDOWN_OVERRIDE_FRAMES fotos seguidas
 *    en contra. Si la visión le da la razón, vuelve a la regla normal.
 *
 * Java puro, sin SDK.
 */
public class HiveBelief {
    private final Alliance alliance;
    private final int confirmFrames, overrideFrames;
    private final long gapNs;

    private Cell upCell;
    private Cell candidate;
    private int streak;
    private int framesNeeded;
    private long lastEvidenceNs = Long.MIN_VALUE;
    private String reason;

    public HiveBelief(Alliance alliance, Cell startUp, int confirmFrames, int overrideFrames, double gapS) {
        this.alliance = alliance;
        this.confirmFrames = Math.max(1, confirmFrames);
        this.overrideFrames = Math.max(1, overrideFrames);
        this.gapNs = (long) (gapS * 1e9);
        this.upCell = startUp;
        this.framesNeeded = this.confirmFrames;
        this.reason = "estado de inicio";
    }

    /**
     * Una foto mide que claimedUp es la de arriba. Fotos que no deciden no se reportan.
     * @param why texto corto para la telemetría
     */
    public void addEvidence(Cell claimedUp, long nowNs, String why) {
        if (claimedUp == null || claimedUp.alliance != alliance) return;
        if (nowNs - lastEvidenceNs > gapNs) {
            candidate = null;
            streak = 0;
        }
        lastEvidenceNs = nowNs;

        if (claimedUp == upCell) {
            candidate = null;
            streak = 0;
            framesNeeded = confirmFrames;      // la visión coincide: regla normal
            reason = why;
            return;
        }
        if (claimedUp == candidate) {
            streak++;
        } else {
            candidate = claimedUp;
            streak = 1;
        }
        if (streak >= framesNeeded) {
            upCell = claimedUp;
            candidate = null;
            streak = 0;
            framesNeeded = confirmFrames;
            reason = why;
        }
    }

    /** El piloto dice cuál está arriba. Gana de inmediato. */
    public void override(Cell up) {
        if (up == null || up.alliance != alliance) return;
        upCell = up;
        candidate = null;
        streak = 0;
        framesNeeded = overrideFrames;
        reason = "piloto";
    }

    /** El piloto avisa que el HIVE se volteó: la de arriba pasa a ser la otra. */
    public void flip() {
        override(upCell.partner());
    }

    public Cell upCell() {
        return upCell;
    }

    public String reason() {
        return reason;
    }
}
