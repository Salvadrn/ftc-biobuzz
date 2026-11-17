package org.firstinspires.ftc.teamcode.util;

/**
 * Guarda unos cuantos números entre OpModes (por ejemplo, dónde quedó el hood).
 * En el robot va a un archivo del Control Hub; en la simulación, a memoria.
 */
public interface StateStore {
    double getDouble(String key, double fallback);
    void putDouble(String key, double value);

    /** Versión en memoria: para la simulación y para cuando el archivo falla. */
    class Memory implements StateStore {
        private final java.util.Map<String, Double> values = new java.util.HashMap<>();

        @Override
        public double getDouble(String key, double fallback) {
            Double v = values.get(key);
            return v == null ? fallback : v;
        }

        @Override
        public void putDouble(String key, double value) {
            values.put(key, value);
        }
    }
}
