package org.firstinspires.ftc.teamcode.feeder;

/**
 * Cuántas pelotas controla el robot (G407: máximo 4 a la vez).
 *
 * Se suma con cada pelota que cuenta el intake, se resta con cada tiro y con cada
 * pelota rechazada. Como respaldo, nunca puede ser menor que las pelotas que los
 * sensores ven en su lugar (arriba, copa, fila y garganta).
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

    public boolean isFull() { return count >= MAX_BALLS; }
}
