package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayDeque;

/**
 * Memoria corta de "a dónde apuntaba la torreta" en cada instante.
 *
 * Por qué existe: la foto del Limelight llega con retraso (30 a 80 ms). Si la torreta
 * se estaba moviendo, el tx de esa foto se midió desde un ángulo que ya no es el actual.
 * Con este historial recuperamos el ángulo que había cuando se tomó la foto y el error
 * se suma al ángulo correcto. Sin esto, una torreta rápida oscila alrededor del objetivo.
 */
public class AngleHistory {
    private static final class Sample {
        final long timeNs;
        final double angleDeg;
        Sample(long timeNs, double angleDeg) { this.timeNs = timeNs; this.angleDeg = angleDeg; }
    }

    private final ArrayDeque<Sample> samples = new ArrayDeque<>();
    private final long windowNs;

    /** @param windowSeconds cuánto tiempo hacia atrás se guarda */
    public AngleHistory(double windowSeconds) {
        this.windowNs = (long) (windowSeconds * 1e9);
    }

    public void add(long timeNs, double angleDeg) {
        samples.addLast(new Sample(timeNs, angleDeg));
        while (samples.size() > 2 && timeNs - samples.peekFirst().timeNs > windowNs) {
            samples.removeFirst();
        }
    }

    /**
     * Ángulo que había en el instante pedido, interpolando entre las dos muestras vecinas.
     * Si el instante queda fuera de lo guardado devuelve la muestra más cercana.
     * Devuelve null si todavía no hay muestras.
     */
    public Double at(long timeNs) {
        if (samples.isEmpty()) return null;

        Sample before = null;
        for (Sample s : samples) {
            if (s.timeNs >= timeNs) {
                if (before == null || s.timeNs == before.timeNs) return s.angleDeg;
                double t = (double) (timeNs - before.timeNs) / (double) (s.timeNs - before.timeNs);
                return before.angleDeg + t * (s.angleDeg - before.angleDeg);
            }
            before = s;
        }
        return before.angleDeg;
    }

    public void clear() { samples.clear(); }
}
