package org.firstinspires.ftc.teamcode.feeder;

/**
 * Cuántas pelotas controla el robot (G407: máximo 4 a la vez).
 *
 * Se suma con cada pelota que cuenta el intake en la garganta, se resta con cada tiro y con
 * cada pelota rechazada. Como respaldo, nunca puede ser menor que las pelotas que los
 * sensores ven en su lugar (arriba, copa y fila).
 *
 * LLENO no es solo la cuenta: con la lengua, la 4a se queda en la boca sin cortar el haz de
 * la garganta (documento del CAD v138, 4.5). Con las 3 etapas ocupadas, una pelota esperando
 * en la entrada es la 4a y el robot está lleno. Esa pelota NO se suma a la cuenta: se cuenta
 * cuando cruza la garganta. Si se sumara aquí, al cruzar después de un tiro se contaría dos
 * veces y el robot se quedaría "lleno" con 3 de ahí en adelante.
 */
public class BallInventory {
    public static final int MAX_BALLS = 4;

    private int count = 0;

    public void entered() { count++; }

    public void left() { count = Math.max(0, count - 1); }

    /** Al arrancar con pelotas precargadas, o si el piloto corrige la cuenta. */
    public void set(int n) { count = Math.max(0, n); }

    /** Lo que ven los sensores es un mínimo seguro. */
    public void reconcile(int sensedAtLeast) {
        if (sensedAtLeast > count) count = sensedAtLeast;
    }

    public int count() { return count; }

    /** Lleno por la cuenta. */
    public boolean isFull() { return count >= MAX_BALLS; }

    /**
     * Lleno por la cuenta, o con las 3 etapas ocupadas (arriba, copa y fila) y otra pelota
     * esperando en la entrada del intake.
     */
    public boolean isFull(int stagesSensed, boolean ballAtEntry) {
        return isFull() || (stagesSensed >= MAX_BALLS - 1 && ballAtEntry);
    }

    /** La 4a está esperando en la entrada sin contarse todavía (para la telemetría). */
    public boolean fourthWaiting(int stagesSensed, boolean ballAtEntry) {
        return !isFull() && isFull(stagesSensed, ballAtEntry);
    }
}
