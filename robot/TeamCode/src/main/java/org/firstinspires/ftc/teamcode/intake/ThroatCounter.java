package org.firstinspires.ftc.teamcode.intake;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.ThroatCfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;

/**
 * La garganta, donde el intake le entrega la pelota a la fila: el break-beam cuenta las
 * pelotas (G407) y el V3 dice qué son (G408). Es la misma para cualquier intake.
 *
 *  - Un corte del haz es pelota nueva solo si antes estuvo libre ThroatCfg.CLEAR_S (una
 *    pelota que rebota no cuenta dos veces).
 *  - Hacia adentro se cuenta al LLEGAR (haz cortado ≥ MIN_S): la 4a pelota se queda parada
 *    sobre el haz y nunca lo libera; contarla al salir la contaría dos veces.
 *  - En reversa del piloto se cuenta al LIBERAR el haz, si se queda libre CLEAR_S.
 *  - Una NECTAR rival se resta solo si ya se había sumado; si no, ya no se suma. Se olvida
 *    cuando el V3 deja de verla y el haz queda libre.
 */
public class ThroatCounter {
    private final BeamSensor beam;
    private final BallSensor color;
    private final Alliance alliance;

    private boolean wasActive = false;
    private boolean counted = false;            // la pelota que está en el haz ya se contó
    private long cutSinceNs;
    private long clearSinceNs = Long.MIN_VALUE / 2;
    private long ejectReleaseNs = -1;
    private boolean rivalPending = false;       // hay una rival en la garganta: no se suma
    private boolean rivalNow = false, newRival = false, justCounted = false;
    private int entered = 0, rejected = 0;
    private BallType lastType = BallType.NONE;

    public ThroatCounter(BeamSensor beam, BallSensor color, Alliance alliance) {
        this.beam = beam;
        this.color = color;
        this.alliance = alliance;
    }

    /**
     * Llamar una vez por ciclo.
     *
     * @param pullingIn el intake está jalando hacia adentro (solo así se cuenta una llegada)
     * @param ejecting  el piloto está sacando pelotas en reversa
     */
    public void update(long nowNs, boolean pullingIn, boolean ejecting) {
        boolean ball = beam.isActive();
        BallType seen = color.read(nowNs);
        rivalNow = seen.isRivalFor(alliance);
        newRival = false;
        justCounted = false;

        if (ball && !wasActive) {
            if ((nowNs - clearSinceNs) / 1e9 >= ThroatCfg.CLEAR_S) {
                cutSinceNs = nowNs;
                counted = false;
            }
            ejectReleaseNs = -1;
        }
        if (!ball && wasActive) {
            clearSinceNs = nowNs;
            if (ejecting && (nowNs - cutSinceNs) / 1e9 >= ThroatCfg.MIN_S) ejectReleaseNs = nowNs;
        }
        if (ejectReleaseNs >= 0 && (nowNs - ejectReleaseNs) / 1e9 >= ThroatCfg.CLEAR_S) {
            ejectReleaseNs = -1;
            rejected++;
        }
        wasActive = ball;

        if (rivalNow && !rivalPending) {
            rivalPending = true;
            newRival = true;
            if (ball && counted) rejected++;
        } else if (!rivalNow && rivalPending && !ball
                && (nowNs - clearSinceNs) / 1e9 >= ThroatCfg.CLEAR_S) {
            rivalPending = false;
        }

        if (ball && !counted && pullingIn && !rivalPending
                && (nowNs - cutSinceNs) / 1e9 >= ThroatCfg.MIN_S) {
            counted = true;
            justCounted = true;
            entered++;
        }
        if (seen.isBall()) lastType = seen;
    }

    /** El V3 acaba de ver una NECTAR rival (una vez por pelota). */
    public boolean newRival() { return newRival; }
    /** El V3 ve una NECTAR rival ahorita. */
    public boolean rivalNow() { return rivalNow; }
    /** Hay una rival en la garganta que todavía no se va. */
    public boolean rivalPending() { return rivalPending; }
    /** En este ciclo se contó una pelota que llegó. */
    public boolean justCounted() { return justCounted; }
    public boolean hasBall() { return beam.isActive(); }
    public BallType lastType() { return lastType; }
    public boolean colorFault() { return color.isFault(); }
    public BallType colorLast() { return color.last(); }

    public int consumeEntered() {
        int n = entered;
        entered = 0;
        return n;
    }

    public int consumeRejected() {
        int n = rejected;
        rejected = 0;
        return n;
    }
}
