package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Cuál de las dos CELLS de nuestra alianza está arriba.
 *
 * El HIVE solo se voltea cuando le meten suficientes pelotas, así que la creencia es
 * "pegajosa": cambia solo cuando varias fotos seguidas dicen lo mismo. El piloto puede
 * forzarla con un botón si la visión se equivoca.
 *
 * Java puro, sin SDK.
 */
public class HiveBelief {
    private final Alliance alliance;
    private final int confirmFrames;

    private Cell upCell;          // null = todavía no se sabe
    private Cell candidate;
    private int streak;
    private String reason = "sin datos";

    public HiveBelief(Alliance alliance, int confirmFrames) {
        this.alliance = alliance;
        this.confirmFrames = Math.max(1, confirmFrames);
    }

    /**
     * Una foto dice que claimedUp es la CELL de arriba (o null si la foto no decide).
     * @param why texto corto para la telemetría
     */
    public void addEvidence(Cell claimedUp, String why) {
        if (claimedUp == null || claimedUp.alliance != alliance) {
            return;
        }
        if (claimedUp == upCell) {
            candidate = null;
            streak = 0;
            reason = why;
            return;
        }
        if (claimedUp == candidate) {
            streak++;
        } else {
            candidate = claimedUp;
            streak = 1;
        }
        if (streak >= confirmFrames) {
            upCell = claimedUp;
            candidate = null;
            streak = 0;
            reason = why;
        }
    }

    /** El piloto dice cuál está arriba. Gana de inmediato. */
    public void override(Cell up) {
        if (up == null || up.alliance != alliance) return;
        upCell = up;
        candidate = null;
        streak = 0;
        reason = "piloto";
    }

    /** El piloto avisa que el HIVE se volteó: la de arriba pasa a ser la otra. */
    public void flip() {
        Cell[] both = Cell.ofAlliance(alliance);
        override(upCell == null ? both[0] : upCell.partner());
    }

    public Cell upCell() {
        return upCell;
    }

    public String reason() {
        return reason;
    }
}
