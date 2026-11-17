package org.firstinspires.ftc.teamcode.vision;

/**
 * Geometría del HIVE y de la cámara. Java puro, sin SDK, para poder probarla en la computadora.
 *
 * Lo que dice el manual TU09 (sección 9.6 y 9.9) y la librería de tags del SDK 12.0:
 *  - Cada HIVE gira en un pivote a 43.95 in del piso y descansa inclinado 30°.
 *    Sus dos CELLS están en las puntas; la de arriba es la única que puntúa.
 *  - La abertura de cada CELL mira hacia afuera a lo largo del HIVE, 30° sobre la
 *    horizontal en la CELL de arriba.
 *  - La fila de 4 tags va en la cara de abajo de cada CELL, mirando hacia abajo y hacia
 *    el lado alto. Las dos CELLS del mismo HIVE quedan con los tags mirando al MISMO
 *    lado: desde donde se tira se ven las dos filas, y hay que saber cuál es la de arriba.
 *  - Con las cotas del manual (abertura de 53.5 a 65.6 in, CELL de 12.04 in, 18.84 in
 *    entre CELLS) la fila de tags de la CELL de arriba queda a unos 49.9 in del piso y
 *    la de abajo a unos 35.6 in. Son 14 in de diferencia: eso es lo que las distingue.
 *  - El centro de la abertura (el origen del cluster en el SDK) queda 3.4 in más afuera
 *    y 8.5 in más arriba que el centro de la fila de tags.
 *
 * Convenciones de la cámara (Limelight): tx positivo = derecha, ty positivo = arriba,
 * medidos sin crosshair (tx_nocross, ty_nocross), en grados.
 */
public final class HiveGeometry {
    private HiveGeometry() {}

    /** Dirección de un objetivo vista desde la cámara, ya sin la inclinación de la cámara. */
    public static final class Ray {
        public final double forward, left, up;   // vector sin normalizar en el marco horizontal
        public final double azimuthLeftDeg;       // + = izquierda del eje de la cámara
        public final double elevationDeg;         // + = arriba de la horizontal

        Ray(double forward, double left, double up) {
            this.forward = forward;
            this.left = left;
            this.up = up;
            this.azimuthLeftDeg = Math.toDegrees(Math.atan2(left, forward));
            this.elevationDeg = Math.toDegrees(Math.atan2(up, Math.hypot(forward, left)));
        }

        public double horizontalNorm() {
            return Math.hypot(forward, left);
        }

        public double norm() {
            return Math.sqrt(forward * forward + left * left + up * up);
        }
    }

    /**
     * Convierte tx, ty de la cámara en un rayo horizontal-vertical real.
     *
     * Con la cámara inclinada, tx no es el ángulo que tiene que girar la torreta: con
     * pitch 30° y ty 10°, un tx de 10° pide girar 12.8°.
     */
    public static Ray ray(double txNcDeg, double tyNcDeg, double cameraPitchDeg) {
        double xn = Math.tan(Math.toRadians(txNcDeg));
        double yUp = Math.tan(Math.toRadians(tyNcDeg));
        double p = Math.toRadians(cameraPitchDeg);
        double forward = Math.cos(p) - yUp * Math.sin(p);
        double up = Math.sin(p) + yUp * Math.cos(p);
        return new Ray(forward, -xn, up);
    }

    /** Lo inverso de ray(): qué tx, ty vería la cámara para un punto en el marco horizontal. */
    public static double[] project(double forward, double left, double up, double cameraPitchDeg) {
        double p = Math.toRadians(cameraPitchDeg);
        // Ejes de la cámara en el marco horizontal
        double zf = Math.cos(p), zu = Math.sin(p);      // eje óptico
        double yf = Math.sin(p), yu = -Math.cos(p);     // "abajo" de la imagen
        double camZ = forward * zf + up * zu;
        double camY = forward * yf + up * yu;
        double camX = -left;
        if (camZ <= 1e-9) return null;                  // detrás de la cámara
        double tx = Math.toDegrees(Math.atan(camX / camZ));
        double ty = Math.toDegrees(Math.atan(-camY / camZ));
        return new double[]{tx, ty};
    }

    /** Distancia horizontal de la cámara a un punto que está heightAboveCamIn más arriba. */
    public static double horizontalDistance(Ray ray, double heightAboveCamIn) {
        if (ray.up <= 1e-9) return Double.NaN;
        return heightAboveCamIn * ray.horizontalNorm() / ray.up;
    }

    /** Distancia en línea recta de la cámara a un punto que está heightAboveCamIn más arriba. */
    public static double slantRange(Ray ray, double heightAboveCamIn) {
        if (ray.up <= 1e-9) return Double.NaN;
        return heightAboveCamIn * ray.norm() / ray.up;
    }

    /**
     * Área esperada de un tag (en % de la imagen, como "ta" del Limelight) a cierta distancia.
     * areaK = 100 · (lado del tag)² · fx² / (ancho · alto de la imagen), en in²; con el
     * campo de visión del Limelight 3A (54.5° x 42°) y tags de 3.25 in da 1327.5.
     * cosView es el coseno del ángulo entre la cara del tag y la cámara.
     */
    public static double expectedAreaPct(double slantRangeIn, double areaK, double cosView) {
        return areaK * cosView / (slantRangeIn * slantRangeIn);
    }

    /**
     * Prueba de tamaño: ¿este tag es de la fila de arriba o de la de abajo?
     *
     * Para el mismo rayo, si el tag fuera de la fila de arriba estaría más lejos y se vería
     * más chico; si fuera de la de abajo, más cerca y más grande. La diferencia es de casi
     * 3 veces en área, así que aguanta errores grandes del modelo.
     *
     * @return puntaje: negativo = parece de arriba, positivo = parece de abajo. Está en
     *         unidades de "medio salto": -1 = justo lo esperado arriba, +1 = justo abajo.
     */
    public static double upDownSizeScore(Ray ray, double measuredAreaPct, double camHeightIn,
                                         double upRowHeightIn, double downRowHeightIn,
                                         double areaK, double cosView) {
        double rUp = slantRange(ray, upRowHeightIn - camHeightIn);
        double rDown = slantRange(ray, downRowHeightIn - camHeightIn);
        if (Double.isNaN(rUp) || Double.isNaN(rDown) || measuredAreaPct <= 0) return Double.NaN;
        double lnUp = Math.log(expectedAreaPct(rUp, areaK, cosView));
        double lnDown = Math.log(expectedAreaPct(rDown, areaK, cosView));
        double mid = 0.5 * (lnUp + lnDown);
        double half = 0.5 * (lnDown - lnUp);       // positivo: la fila de abajo se ve más grande
        return (Math.log(measuredAreaPct) - mid) / half;
    }

    /** Altura de un punto dado en el espacio 3D de la cámara del Limelight (x der, y abajo, z al frente). */
    public static double heightFromCameraSpace(double yDownIn, double zForwardIn,
                                               double camHeightIn, double cameraPitchDeg) {
        double p = Math.toRadians(cameraPitchDeg);
        return camHeightIn + zForwardIn * Math.sin(p) - yDownIn * Math.cos(p);
    }

    /**
     * Pasa un objetivo visto desde la cámara al eje de la torreta.
     * La cámara va detrás del eje (en el CAD v99, 76 mm = 2.99 in) y sobre la línea del
     * lanzador, así que al centrar el objetivo los dos apuntan igual; en medio, no.
     *
     * @return {rumbo desde el eje en grados (+ izquierda), distancia horizontal desde el eje}
     */
    public static double[] fromCameraToTurretAxis(double azimuthLeftDeg, double horizontalDistIn,
                                                  double cameraBehindAxisIn) {
        double a = Math.toRadians(azimuthLeftDeg);
        double fwd = horizontalDistIn * Math.cos(a) - cameraBehindAxisIn;
        double left = horizontalDistIn * Math.sin(a);
        return new double[]{Math.toDegrees(Math.atan2(left, fwd)), Math.hypot(fwd, left)};
    }
}
