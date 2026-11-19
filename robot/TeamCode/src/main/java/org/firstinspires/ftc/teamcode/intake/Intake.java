package org.firstinspires.ftc.teamcode.intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallType;

/**
 * Lo que el resto del robot necesita de un intake, sin importar el mecanismo.
 *
 * El intake ya cambió una vez (el rotor F2 salió en la v106 del CAD y entró la lengua de
 * bandas). Todo lo demás (TeleOp, alimentador, cuenta de pelotas) habla con esta interfaz,
 * así que cambiar el mecanismo es escribir otra clase que la implemente y cambiar una línea
 * en Robot.
 */
public interface Intake {
    enum Command { STOP, IN, OUT }

    /**
     * Preparación en INIT (R103.B): dejarlo dentro del tamaño de inicio (R102).
     * Se llama en cada ciclo de INIT; regresa true cuando terminó.
     */
    boolean initStep(long nowNs);

    /**
     * Quedó dentro del tamaño de inicio (R102). Puede ser false aunque initStep ya haya
     * terminado: por ejemplo, si no encontró el índice o no llegó a 0°.
     */
    boolean isStartReady();

    /** Lo que pide el piloto. El intake puede no obedecer (lleno, atasco, NECTAR rival). */
    void setCommand(Command command);

    /**
     * Llamar una vez por ciclo.
     * @param robotFull ya hay 4 pelotas adentro (G407): no jalar más
     */
    void update(long nowNs, boolean robotFull);

    /** Está metiendo pelotas: el alimentador puede jalar la siguiente a la fila. */
    boolean isFeeding();

    /** Pide reversa de la fila (para sacar una NECTAR rival, G408). */
    boolean wantsFilaReverse();

    /** Pide que la fila no se mueva: hay una NECTAR rival atorada en la entrada (G408). */
    default boolean holdsFila() { return false; }

    /** Pelotas que entraron desde la última consulta (para G407). */
    int consumeEntered();

    /** Pelotas que se sacaron (rechazadas) desde la última consulta. */
    int consumeRejected();

    /** ¿Hay una pelota parada en la entrada del intake ahorita? (para la cuenta mínima) */
    boolean hasBallAtThroat();

    /** Tipo de la última pelota que entró. */
    BallType lastBallType();

    /**
     * El piloto confirma que lo que ve la cámara de la visera no es una NECTAR rival y
     * quiere jalar de todos modos. Intakes sin cámara lo ignoran.
     */
    default void setRivalOverride(boolean ignoreCamera) {}

    /** Aviso para el piloto, o "" si no hay. */
    String warning();

    void stop();

    void addTelemetry(Telemetry telemetry);
}
