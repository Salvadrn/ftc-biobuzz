package org.firstinspires.ftc.teamcode.feeder;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;

/**
 * Camino de la pelota del CAD v99, desde la fila hasta el flywheel:
 *
 *   fila (rodillo con servo continuo, break-beam BB3)
 *     -> copa (aro en cremallera, servo de posición, 0 a 54 mm; V3 de z 50)
 *     -> etapa de arriba (Gecko de z 128 movida por el motor del elevador y una loca
 *        con resorte; V3 de z 126)
 *     -> flywheel.
 *
 * Una pelota por etapa (confirmado por la sesión del CAD el 17-nov):
 *  - Disparar = prender el elevador hacia arriba hasta que la pelota de arriba se vaya.
 *  - Rellenar = la copa sube la pelota de abajo hasta las Gecko de z 95, el elevador a
 *    baja potencia la sube hasta que el V3 de arriba la vea, y la copa baja a 0.
 *  - Con la copa ABAJO, las Gecko no tocan la pelota de la copa: disparar no la mueve.
 *  - La copa NUNCA sube si hay pelota esperando arriba (las dos se tocarían).
 *  - La fila carga la copa solo con la copa abajo.
 *
 * Fallas que se cuidan (revisión del 17-nov):
 *  - Un V3 sin lectura cuenta como OCUPADO (BallSensor da UNKNOWN): la copa no sube.
 *  - Un tiro solo se cuenta si el flywheel lo sintió (caída de rpm) y el V3 de arriba ya
 *    no ve la pelota. Si el V3 la pierde y el flywheel no siente nada, la pelota puede ir
 *    atorada en el hood: no se rellena y el gap no se mueve hasta que el artillero la
 *    empuje (forceShot) o diga que el camino está libre (release).
 *  - Rellenar que falla REFILL_MAX_FAILS veces seguidas se detiene. Si la copa regresa
 *    vacía de un rellenado fallido, la pelota se quedó en el tubo: se detiene a la primera.
 *  - La fila atorada se apaga y reintenta cuando algo cambia o tras FILA_JAM_COOLDOWN_S.
 *  - El elevador no empuja si el flywheel está apagado.
 *
 * Servos con setPwmRange y setPwmDisable() como pide la guía del CAD: la copa se apaga
 * abajo (el piso carga el peso) y la fila se apaga para parar (en 0 se arrastra).
 */
public class Feeder {
    public enum State { IDLE, FIRING, COPA_UP, LIFT, COPA_DOWN }

    private enum FilaMode { STOP, IN, OUT }

    private final CRServo fila;
    private final Servo copa;
    private final DcMotor elevator;
    private final BallSensor upperSensor;
    private final BallSensor copaSensor;
    private final BeamSensor filaBeam;

    private State state = State.IDLE;
    private long stateStartNs;

    // Copa: comando en rampa y estimado de dónde va el servo
    private double copaCmdMm = 0, copaTargetMm = 0, copaEstMm = 0;
    private boolean copaPwmOn = true;
    private long copaStillSinceNs = 0;
    private long copaSeatedSinceNs = -1;

    // Disparo
    private boolean shotRequested = false, forceRequested = false;
    private boolean forced = false;                // este tiro arrancó sin pelota vista arriba
    private BallType firingBall = BallType.NONE;   // la que está saliendo: su gap no se toca
    private long dipNs = -1, upperLostNs = -1;
    private int shotsDone = 0;
    private boolean clearPending = false;          // tiro sin confirmar: puede haber pelota en el hood

    // Rellenar
    private int refillFails = 0;
    private boolean refillBlocked = false;
    private boolean liftCheck = false;             // falló un rellenado: ¿regresó la pelota a la copa?

    // Fila
    private FilaMode filaMode = FilaMode.STOP;
    private boolean filaCommanded = false;
    private long filaPushStartNs = -1;
    private long filaUnjamUntilNs = -1;
    private int filaUnjamTries = 0;
    private boolean filaJammed = false;
    private long filaJamSinceNs = -1;
    private boolean jamBeamWas, jamCopaWas;

    private String warning = "";
    private long lastUpdateNs = 0;

    public Feeder(CRServo fila, Servo copa, DcMotor elevator,
                  BallSensor upperSensor, BallSensor copaSensor, BeamSensor filaBeam) {
        this.fila = fila;
        this.copa = copa;
        this.elevator = elevator;
        this.upperSensor = upperSensor;
        this.copaSensor = copaSensor;
        this.filaBeam = filaBeam;

        if (fila instanceof PwmControl) {
            ((PwmControl) fila).setPwmRange(new PwmControl.PwmRange(FeederCfg.FILA_PWM_MIN_US, FeederCfg.FILA_PWM_MAX_US));
        }
        if (copa instanceof PwmControl) {
            ((PwmControl) copa).setPwmRange(new PwmControl.PwmRange(FeederCfg.COPA_UP_US, FeederCfg.COPA_DOWN_US));
        }
        elevator.setDirection(FeederCfg.ELEVATOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0);
        setFila(FilaMode.STOP);
    }

    /** INIT: copa abajo y suelta, fila y elevador quietos. */
    public void initHardware(long nowNs) {
        copaCmdMm = copaTargetMm = copaEstMm = 0;
        writeCopa();
        copaStillSinceNs = nowNs;
        elevator.setPower(0);
        setFila(FilaMode.STOP);
        stateStartNs = nowNs;
        lastUpdateNs = nowNs;
    }

    /** En INIT: solo baja y suelta la copa. No corre la máquina de estados. */
    public void initUpdate(long nowNs) {
        double dt = lastUpdateNs == 0 ? 0 : Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;
        upperSensor.read(nowNs);
        copaSensor.read(nowNs);
        updateCopa(nowNs, dt);
    }

    /** Pide un tiro. Solo se ejecuta si hay pelota arriba y el mecanismo está libre. */
    public void requestShot() {
        shotRequested = true;
    }

    /**
     * Empuja con el elevador aunque el V3 de arriba no vea pelota: para sacar una pelota que
     * quedó atorada entre la etapa de arriba y el flywheel. Con la copa abajo, como siempre.
     */
    public void forceShot() {
        forceRequested = true;
    }

    public void cancelShot() {
        shotRequested = false;
        forceRequested = false;
    }

    /**
     * El artillero dice que el camino está libre (B en gamepad 2): quita el bloqueo de un
     * tiro sin confirmar, de rellenar y de la fila atorada.
     */
    public void release() {
        clearPending = false;
        refillBlocked = false;
        refillFails = 0;
        liftCheck = false;
        filaJammed = false;
        filaUnjamTries = 0;
        warning = "";
    }

    /**
     * Llamar una vez por ciclo.
     *
     * @param intakeFeeding el intake está metiendo pelotas (la fila puede jalar la siguiente)
     * @param filaReverse   el intake pide reversa de la fila (NECTAR rival, G408)
     * @param filaHold      el intake pide que la fila no se mueva (NECTAR rival atorada)
     * @param shotDip       el flywheel acaba de perder rpm de golpe (una pelota pasó)
     * @param flywheelOn    el flywheel está prendido: sin él no se empuja nada hacia arriba
     */
    public void update(long nowNs, boolean intakeFeeding, boolean filaReverse, boolean filaHold,
                       boolean shotDip, boolean flywheelOn) {
        double dt = lastUpdateNs == 0 ? 0 : Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;

        BallType upper = upperSensor.read(nowNs);
        BallType inCopa = copaSensor.read(nowNs);
        boolean upperFault = upperSensor.isFault();
        boolean upperSeen = upper.isBall() && !upperFault;   // la ve de verdad
        boolean upperOccupied = upper.isBall();              // vista o sin lectura: no subir nada
        boolean copaDown = copaIsDown();

        if (inCopa.isBall() && copaDown) {
            if (copaSeatedSinceNs < 0) copaSeatedSinceNs = nowNs;
        } else {
            copaSeatedSinceNs = -1;
        }
        boolean copaSeated = copaSeatedSinceNs >= 0
                && (nowNs - copaSeatedSinceNs) / 1e9 >= FeederCfg.COPA_SEATED_S;

        switch (state) {
            case IDLE: {
                elevator.setPower(0);
                if (liftCheck && secondsIn(nowNs) >= FeederCfg.LIFT_CHECK_S) {
                    liftCheck = false;
                    if (!inCopa.isBall()) {
                        refillBlocked = true;
                        warning = "una pelota se quedó en el tubo entre la copa y arriba";
                    }
                }
                boolean wantsShot = (shotRequested && upperOccupied) || forceRequested;
                if (wantsShot && copaDown && flywheelOn) {
                    forced = forceRequested && !upperSeen;
                    if (upperOccupied) firingBall = upper;
                    else if (!clearPending) firingBall = BallType.UNKNOWN;   // gap de NECTAR: el seguro
                    dipNs = -1;
                    upperLostNs = -1;
                    enter(State.FIRING, nowNs);
                } else if (!upperOccupied && copaSeated && !liftCheck && !refillBlocked && !clearPending) {
                    copaTargetMm = FeederCfg.COPA_TRAVEL_MM;   // la fila se detiene sola al subir la copa
                    enter(State.COPA_UP, nowNs);
                }
                break;
            }

            case FIRING: {
                if (shotDip) dipNs = nowNs;
                if (upperSeen) {
                    upperLostNs = -1;   // la pérdida tiene que ser continua
                    if (dipNs >= 0 && (nowNs - dipNs) / 1e9 > FeederCfg.DIP_EARLY_S) dipNs = -1;
                } else if (upperLostNs < 0) {
                    upperLostNs = nowNs;
                }
                boolean doneByDip = dipNs >= 0 && !upperSeen
                        && (nowNs - dipNs) / 1e9 >= FeederCfg.FIRE_AFTER_DIP_S;
                boolean lostNoDip = !forced && !upperFault && dipNs < 0 && upperLostNs >= 0
                        && (nowNs - upperLostNs) / 1e9 >= FeederCfg.FIRE_AFTER_LOST_S;
                if (doneByDip || lostNoDip || !flywheelOn || secondsIn(nowNs) >= FeederCfg.FIRE_TIMEOUT_S) {
                    finishShot(nowNs, upperSeen, upperFault, flywheelOn);
                } else {
                    elevator.setPower(FeederCfg.ELEVATOR_FIRE_POWER);
                }
                break;
            }

            case COPA_UP:
                elevator.setPower(0);
                if (upperOccupied) {                        // apareció pelota arriba: no subir
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                } else if (copaArrived(nowNs)) {
                    enter(State.LIFT, nowNs);
                } else if (secondsIn(nowNs) >= FeederCfg.COPA_MOVE_TIMEOUT_S) {
                    refillFailed("la copa no llegó arriba");
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                }
                break;

            case LIFT:
                if (upperOccupied) {
                    elevator.setPower(0);
                    if (upperSeen) refillFails = 0;
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                } else if (secondsIn(nowNs) >= FeederCfg.LIFT_TIMEOUT_S) {
                    elevator.setPower(0);
                    refillFailed("la pelota no subió a la etapa de arriba");
                    liftCheck = true;
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                } else {
                    elevator.setPower(FeederCfg.ELEVATOR_STAGE_POWER);
                }
                break;

            case COPA_DOWN:
                elevator.setPower(0);
                if (copaArrived(nowNs)) enter(State.IDLE, nowNs);
                break;
        }

        updateCopa(nowNs, dt);
        updateFila(nowNs, intakeFeeding, filaReverse, filaHold, inCopa.isBall());
    }

    private void finishShot(long nowNs, boolean upperSeen, boolean upperFault, boolean flywheelOn) {
        elevator.setPower(0);
        shotRequested = false;
        forceRequested = false;
        if (dipNs >= 0 && !upperSeen) {             // el flywheel la sintió y arriba ya no está
            shotsDone++;
            clearPending = false;
            warning = "";
        } else if (upperSeen) {
            warning = flywheelOn ? "el tiro no salió" : "se apagó el flywheel: el tiro no salió";
        } else if (forced) {
            warning = "no salió nada";
        } else {
            clearPending = true;
            warning = upperFault ? "tiro sin confirmar (V3 de arriba sin lectura)" : "tiro sin confirmar";
        }
        enter(State.IDLE, nowNs);
    }

    private void refillFailed(String why) {
        refillFails++;
        if (refillFails >= FeederCfg.REFILL_MAX_FAILS) refillBlocked = true;
        warning = why;
    }

    // ---------- copa ----------

    private void updateCopa(long nowNs, double dt) {
        double before = copaCmdMm;
        if (copaTargetMm > copaCmdMm) {
            double slowFrom = FeederCfg.COPA_TRAVEL_MM - FeederCfg.COPA_SLOW_ZONE_MM;
            double rate = copaCmdMm >= slowFrom ? FeederCfg.COPA_SLOW_MM_S : FeederCfg.COPA_FAST_MM_S;
            copaCmdMm = Math.min(copaTargetMm, copaCmdMm + rate * dt);
            if (before < slowFrom && copaCmdMm > slowFrom) copaCmdMm = slowFrom;   // no brincar la zona lenta
        } else if (copaTargetMm < copaCmdMm) {
            copaCmdMm = Math.max(copaTargetMm, copaCmdMm - FeederCfg.COPA_FAST_MM_S * dt);
        }

        double step = FeederCfg.COPA_SERVO_MM_S * dt;
        copaEstMm += Range.clip(copaCmdMm - copaEstMm, -step, step);
        if (copaCmdMm != before || Math.abs(copaEstMm - copaCmdMm) > 0.5) copaStillSinceNs = nowNs;

        boolean restingDown = copaTargetMm == 0 && copaCmdMm == 0
                && (nowNs - copaStillSinceNs) / 1e9 >= FeederCfg.COPA_DOWN_RELAX_S;
        if (restingDown) {
            if (copaPwmOn && copa instanceof PwmControl) {
                ((PwmControl) copa).setPwmDisable();
                copaPwmOn = false;
            }
        } else {
            writeCopa();
        }
    }

    private void writeCopa() {
        if (!copaPwmOn && copa instanceof PwmControl) {
            ((PwmControl) copa).setPwmEnable();
        }
        copaPwmOn = true;
        // Posición 0 = arriba (54 mm), 1 = abajo (0 mm): así está el rango PWM
        copa.setPosition(Range.clip(1.0 - copaCmdMm / FeederCfg.COPA_TRAVEL_MM, 0, 1));
    }

    private boolean copaArrived(long nowNs) {
        return copaCmdMm == copaTargetMm && Math.abs(copaEstMm - copaCmdMm) <= 0.5
                && (nowNs - copaStillSinceNs) / 1e9 >= FeederCfg.COPA_SETTLE_S;
    }

    private boolean copaIsDown() {
        return copaTargetMm == 0 && copaCmdMm == 0 && copaEstMm <= 0.5;
    }

    // ---------- fila ----------

    private void updateFila(long nowNs, boolean intakeFeeding, boolean filaReverse, boolean filaHold,
                            boolean copaHasBall) {
        boolean filaHasBall = filaBeam.isActive();
        boolean copaBusy = state == State.COPA_UP || state == State.LIFT || state == State.COPA_DOWN
                || !copaIsDown() || liftCheck;

        // Atorada: se queda apagada hasta que algo cambie o pase el tiempo de espera
        if (filaJammed) {
            boolean changed = filaHasBall != jamBeamWas || copaHasBall != jamCopaWas;
            if (changed || filaReverse || (nowNs - filaJamSinceNs) / 1e9 >= FeederCfg.FILA_JAM_COOLDOWN_S) {
                filaJammed = false;
                filaUnjamTries = 0;
            }
        }

        FilaMode want;
        if (filaReverse) {
            want = FilaMode.OUT;
        } else if (filaUnjamUntilNs > nowNs) {
            want = FilaMode.OUT;
        } else if (filaHold || copaBusy || filaJammed) {
            want = FilaMode.STOP;
        } else if (!copaHasBall && (filaHasBall || intakeFeeding)) {
            want = FilaMode.IN;                    // cargar la copa
        } else if (copaHasBall && !filaHasBall && intakeFeeding) {
            want = FilaMode.IN;                    // jalar la siguiente a la fila
        } else {
            want = FilaMode.STOP;
        }

        // Atasco: la fila empuja una pelota a la copa vacía y la copa no la recibe
        boolean pushingToCopa = want == FilaMode.IN && !copaHasBall && filaHasBall;
        if (pushingToCopa) {
            if (filaPushStartNs < 0) filaPushStartNs = nowNs;
            if ((nowNs - filaPushStartNs) / 1e9 >= FeederCfg.FILA_JAM_S) {
                filaPushStartNs = -1;
                if (filaUnjamTries < FeederCfg.FILA_UNJAM_MAX_TRIES) {
                    filaUnjamTries++;
                    filaUnjamUntilNs = nowNs + (long) (FeederCfg.FILA_UNJAM_S * 1e9);
                    want = FilaMode.OUT;
                } else {
                    filaJammed = true;
                    filaJamSinceNs = nowNs;
                    jamBeamWas = filaHasBall;
                    jamCopaWas = copaHasBall;
                    want = FilaMode.STOP;
                }
            }
        } else {
            filaPushStartNs = -1;
            if (copaHasBall) filaUnjamTries = 0;
        }

        setFila(want);
    }

    /**
     * Solo manda comandos cuando cambia el modo. Ojo con el SDK (LynxServoController, leído
     * con javap): apagar el PWM borra su caché y el siguiente setPower, aunque sea 0, lo
     * vuelve a prender. Por eso en STOP se manda 0 primero, se apaga después y ya no se
     * toca hasta el siguiente cambio. En 0 con el PWM prendido la fila se arrastra (banda
     * muerta de 4 µs).
     */
    private void setFila(FilaMode mode) {
        if (mode == filaMode && filaCommanded) return;
        filaMode = mode;
        filaCommanded = true;
        if (mode == FilaMode.STOP) {
            fila.setPower(0);
            if (fila instanceof PwmControl) ((PwmControl) fila).setPwmDisable();
            return;
        }
        if (fila instanceof PwmControl) ((PwmControl) fila).setPwmEnable();
        fila.setPower(mode == FilaMode.IN ? FeederCfg.FILA_POWER_IN : -FeederCfg.FILA_POWER_IN);
    }

    // ---------- estado ----------

    private void enter(State s, long nowNs) {
        state = s;
        stateStartNs = nowNs;
    }

    private double secondsIn(long nowNs) {
        return (nowNs - stateStartNs) / 1e9;
    }

    /** Tiros confirmados desde la última consulta (para la cuenta de G407). */
    public int consumeShots() {
        int s = shotsDone;
        shotsDone = 0;
        return s;
    }

    public State getState() { return state; }
    public boolean isBusy() { return state != State.IDLE; }
    /** Hay pelota arriba (o el V3 de arriba no lee: entonces se supone que sí). */
    public boolean hasUpperBall() { return upperSensor.hasBall(); }
    public BallType upperBall() { return upperSensor.last(); }
    public BallType copaBall() { return copaSensor.last(); }
    public boolean filaHasBall() { return filaBeam.isActive(); }
    public double getCopaMm() { return copaCmdMm; }
    public String getWarning() { return warning; }
    public boolean isClearPending() { return clearPending; }
    public boolean isRefillBlocked() { return refillBlocked; }
    public boolean isFilaJammed() { return filaJammed; }

    /**
     * La pelota para la que debe estar el hood. Mientras dispara, o si un tiro quedó sin
     * confirmar, la que salió: el V3 de arriba la deja de ver cuando todavía va por el hood,
     * y cambiar el gap con una NECTAR adentro la apretaría. Si no, la de arriba; si no hay,
     * la de la copa.
     */
    public BallType nextBall() {
        if (state == State.FIRING || clearPending) return firingBall;
        return upperSensor.hasBall() ? upperSensor.last() : copaSensor.last();
    }

    /** Pelotas que los sensores ven en su lugar (arriba, copa, fila). Sin lectura cuenta como pelota. */
    public int sensedBalls() {
        return (upperSensor.hasBall() ? 1 : 0) + (copaSensor.hasBall() ? 1 : 0) + (filaBeam.isActive() ? 1 : 0);
    }

    public void stop() {
        elevator.setPower(0);
        setFila(FilaMode.STOP);
        shotRequested = false;
        forceRequested = false;
    }

    private static String describe(BallSensor s) {
        return s.isFault() ? "SIN LECTURA" : s.last().toString();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Alimentador", "%s  arriba %s  copa %s (%.0f mm)  fila %s",
                state, describe(upperSensor), describe(copaSensor), copaCmdMm, filaBeam.isActive() ? "sí" : "no");
        if (clearPending) {
            telemetry.addLine("!! tiro sin confirmar: no se rellena. Gatillo izq. (2) empuja; B (2) si ya está libre");
        }
        if (refillBlocked) telemetry.addLine("!! rellenar detenido: revisa el tubo; B (2) para reintentar");
        if (filaJammed) telemetry.addLine("!! la fila está atorada: apagada, reintenta sola");
        if (upperSensor.isFault() || copaSensor.isFault()) {
            telemetry.addLine("!! V3 sin lectura (" + (upperSensor.isFault() ? "arriba " : "")
                    + (copaSensor.isFault() ? "copa" : "") + "): revisa el cable");
        }
        if (!warning.isEmpty()) telemetry.addLine("!! " + warning);
    }
}
