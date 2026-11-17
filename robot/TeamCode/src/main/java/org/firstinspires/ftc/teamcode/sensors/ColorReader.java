package org.firstinspires.ftc.teamcode.sensors;

/**
 * Lo mínimo que se necesita de un sensor de color: r, g, b normalizados y distancia en mm.
 * Separarlo del SDK permite simular los sensores en la computadora.
 */
public interface ColorReader {
    /** Llena out = {r, g, b, distanciaMm}. Regresa false si la lectura falló. */
    boolean read(double[] out);
}
