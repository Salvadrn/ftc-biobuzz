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
 *    Se confirma por la caída de rpm del flywheel y, de apoyo, porque el V3 de arriba
 *    la deja de ver. Con tiempo máximo.
 *  - Rellenar = la copa sube la pelota de abajo hasta las Gecko de z 95, el elevador a
 *    baja potencia la sube hasta que el V3 de arriba la vea, y la copa baja a 0.
 *  - Con la copa ABAJO, las Gecko no tocan la pelota de la copa: disparar no la mueve.
 *  - La copa NUNCA sube si hay pelota esperando arriba (las dos se tocarían).
 *  - La fila carga la copa solo con la copa abajo.
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
    private boolean shotRequested = false;
    private long dipNs = -1, upperLostNs = -1;
    private int shotsDone = 0;

    // Fila
    private FilaMode filaMode = FilaMode.STOP;
    private boolean filaPwmOn = true;
    private long filaPushStartNs = -1;
    private long filaUnjamUntilNs = -1;
    private int filaUnjamTries = 0;

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

    public void cancelShot() {
        shotRequested = false;
    }

    /**
     * Llamar una vez por ciclo.
     *
     * @param intakeFeeding   el intake está metiendo pelotas (la fila puede jalar la siguiente)
     * @param rejectRequested el intake pide reversa de la fila (NECTAR rival, G408)
     * @param shotDip         el flywheel acaba de perder rpm de golpe (una pelota pasó)
     */
    public void update(long nowNs, boolean intakeFeeding, boolean rejectRequested, boolean shotDip) {
        double dt = lastUpdateNs == 0 ? 0 : Range.clip((nowNs - lastUpdateNs) / 1e9, 0, 0.1);
        lastUpdateNs = nowNs;

        BallType upper = upperSensor.read(nowNs);
        BallType inCopa = copaSensor.read(nowNs);
        boolean copaDown = copaIsDown(nowNs);

        if (inCopa.isBall() && copaDown) {
            if (copaSeatedSinceNs < 0) copaSeatedSinceNs = nowNs;
        } else {
            copaSeatedSinceNs = -1;
        }
        boolean copaSeated = copaSeatedSinceNs >= 0
                && (nowNs - copaSeatedSinceNs) / 1e9 >= FeederCfg.COPA_SEATED_S;

        switch (state) {
            case IDLE:
                if (shotRequested && upper.isBall() && copaDown) {
                    enter(State.FIRING, nowNs);
                    dipNs = -1;
                    upperLostNs = -1;
                } else if (!upper.isBall() && copaSeated) {          // la fila se detiene sola al subir la copa
                    enter(State.COPA_UP, nowNs);
                    copaTargetMm = FeederCfg.COPA_TRAVEL_MM;
                }
                elevator.setPower(0);
                break;

            case FIRING:
                elevator.setPower(FeederCfg.ELEVATOR_FIRE_POWER);
                if (shotDip && dipNs < 0) dipNs = nowNs;
                if (!upper.isBall() && upperLostNs < 0) upperLostNs = nowNs;
                boolean doneByDip = dipNs >= 0 && (nowNs - dipNs) / 1e9 >= FeederCfg.FIRE_AFTER_DIP_S;
                boolean doneByLoss = upperLostNs >= 0 && (nowNs - upperLostNs) / 1e9 >= FeederCfg.FIRE_AFTER_LOST_S;
                boolean timeout = secondsIn(nowNs) >= FeederCfg.FIRE_TIMEOUT_S;
                if (doneByDip || doneByLoss || timeout) {
                    elevator.setPower(0);
                    shotRequested = false;
                    if (dipNs >= 0 || upperLostNs >= 0) {
                        shotsDone++;
                        warning = "";
                    } else {
                        warning = "el tiro no salió";
                    }
                    enter(State.IDLE, nowNs);
                }
                break;

            case COPA_UP:
                elevator.setPower(0);
                if (upper.isBall()) {                       // apareció pelota arriba: no subir
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                } else if (copaArrived(nowNs)) {
                    enter(State.LIFT, nowNs);
                } else if (secondsIn(nowNs) >= FeederCfg.COPA_MOVE_TIMEOUT_S) {
                    warning = "la copa no llegó arriba";
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                }
                break;

            case LIFT:
                elevator.setPower(FeederCfg.ELEVATOR_STAGE_POWER);
                if (upper.isBall()) {
                    elevator.setPower(0);
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                } else if (secondsIn(nowNs) >= FeederCfg.LIFT_TIMEOUT_S) {
                    elevator.setPower(0);
                    warning = "la pelota no subió a la etapa de arriba";
                    copaTargetMm = 0;
                    enter(State.COPA_DOWN, nowNs);
                }
                break;

            case COPA_DOWN:
                elevator.setPower(0);
                if (copaArrived(nowNs)) enter(State.IDLE, nowNs);
                break;
        }

        updateCopa(nowNs, dt);
        updateFila(nowNs, intakeFeeding, rejectRequested, inCopa.isBall());
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

    private boolean copaIsDown(long nowNs) {
        return copaTargetMm == 0 && copaCmdMm == 0 && copaEstMm <= 0.5;
    }

    // ---------- fila ----------

    private void updateFila(long nowNs, boolean intakeFeeding, boolean rejectRequested, boolean copaHasBall) {
        boolean filaHasBall = filaBeam.isActive();
        boolean copaBusy = state == State.COPA_UP || state == State.LIFT || state == State.COPA_DOWN
                || !copaIsDown(nowNs);

        FilaMode want;
        if (rejectRequested) {
            want = FilaMode.OUT;
        } else if (filaUnjamUntilNs > nowNs) {
            want = FilaMode.OUT;
        } else if (copaBusy) {
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
                    warning = "la fila está atorada";
                }
            }
        } else {
            filaPushStartNs = -1;
            if (copaHasBall) filaUnjamTries = 0;
        }

        setFila(want);
    }

    private void setFila(FilaMode mode) {
        filaMode = mode;
        if (mode == FilaMode.STOP) {
            fila.setPower(0);
            if (filaPwmOn && fila instanceof PwmControl) {
                ((PwmControl) fila).setPwmDisable();   // en 0 se arrastra por la banda muerta de 4 µs
            }
            filaPwmOn = false;
            return;
        }
        if (!filaPwmOn && fila instanceof PwmControl) {
            ((PwmControl) fila).setPwmEnable();
        }
        filaPwmOn = true;
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
    public boolean hasUpperBall() { return upperSensor.hasBall(); }
    public BallType upperBall() { return upperSensor.last(); }
    public BallType copaBall() { return copaSensor.last(); }
    public boolean filaHasBall() { return filaBeam.isActive(); }
    public double getCopaMm() { return copaCmdMm; }
    public String getWarning() { return warning; }

    /** La pelota que sigue en salir: la de arriba; si no hay, la de la copa. */
    public BallType nextBall() {
        return upperSensor.hasBall() ? upperSensor.last() : copaSensor.last();
    }

    /** Pelotas que los sensores ven en su lugar (arriba, copa, fila). */
    public int sensedBalls() {
        return (upperSensor.hasBall() ? 1 : 0) + (copaSensor.hasBall() ? 1 : 0) + (filaBeam.isActive() ? 1 : 0);
    }

    public void stop() {
        elevator.setPower(0);
        setFila(FilaMode.STOP);
        shotRequested = false;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Alimentador", "%s  arriba %s  copa %s (%.0f mm)  fila %s",
                state, upperSensor.last(), copaSensor.last(), copaCmdMm, filaBeam.isActive() ? "sí" : "no");
        if (!warning.isEmpty()) telemetry.addLine("!! " + warning);
    }
}
