package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.vision.RivalNectarDetector;

import java.util.Random;

/**
 * Intake de la lengua de bandas con dedos colgantes (CAD v107-v118, revisada en la
 * v138-v139). M_F mueve las bandas y los 4 dedos; un servo 2000-0025-0002 con manivela y
 * biela despliega la lengua. No hay estacionado: ni imán ni encoder.
 *
 * La lengua tiene dos posiciones y no tiene sensor; su servo y la secuencia para plegarla y
 * desplegarla (rápido hasta el empuje y luego en rampa) están en TongueServo.
 *  - SIN CALIBRAR (IntakeLenguaCfg.CALIBRATED = false) no se mueve nada, ni la lengua ni M_F:
 *    los µs de cálculo pueden estar hasta ±48 µs corridos por el calado del estriado. En
 *    INIT avisa que se pliegue a mano. Solo "Tune: Lengua" la mueve.
 *  - INIT: a plegada y luego PWM apagado; se sostiene sola contra el tope de atrás (R103) y
 *    el robot queda quieto (G304.H).
 *  - Solo se pliega con la boca (BB1) y la garganta LIBRES: una pelota bajo los dedos
 *    centrales los levanta y, al subir, chocan con la flecha del pivote de la fila (desde
 *    ~12° de lengua con NECTAR y ~25° con POLLEN). La 4a puede quedarse bajo los dedos
 *    cortando solo BB1 (documento del CAD v138, 4.6).
 *  - Al dar Play se queda plegada. Baja cuando el intake está prendido y no está lleno
 *    (decidido por Adrián el 19-nov, con la precarga de 3 adentro + 1 en la loseta).
 *  - Con 4 pelotas (G407) se queda ABAJO y M_F para en BRAKE.
 *  - Si la fila no está jalando (tiene pelota, o la copa está ocupada o moviéndose), una
 *    pelota en la boca o en la garganta ESPERA: M_F para en BRAKE (no la empuja contra la
 *    pelota de la fila) y no cuenta como atasco. Con las 3 etapas ocupadas, esa es la 4a (4.5).
 *  - Abajo, PWM prendido. M_F solo jala con la lengua abajo.
 *
 * Jalar al 100 %, atasco por BB1, cuenta en la garganta (ThroatCounter) y G408 con la
 * cámara y el V3 de la garganta. Para regresar una rival la lengua tiene que estar ABAJO:
 * si estaba plegada, primero baja, con la fila quieta, y luego reversa de M_F y de la fila.
 *
 * M_F con límite de corriente (plan de la lengua, ~5 A): arriba del límite MF_OVERCURRENT_S
 * seguidos, se detiene en BRAKE y avisa; vuelve a intentar cuando el piloto cambia lo que
 * pide. La lengua no se mueve por eso. Ojo: en reversa la lengua puede subir hasta 34° por la
 * ranura de la biela, así que "abajo" no garantiza que esté en su tope.
 */
public class IntakeLengua implements Intake {
    private enum State { IDLE, PULLING, UNJAM, BLOCKED, REJECTING, RIVAL_STUCK, EJECTING, FULL, STALLED }

    private static final String UNCALIBRATED =
            "lengua sin calibrar (IntakeLenguaCfg.CALIBRATED): no se mueve ni la lengua ni M_F. "
                    + "Pliégala a mano y pásala por \"Tune: Lengua\"";

    private final DcMotorEx motor;
    private final TongueServo tongue;
    private final BeamSensor mouth;
    private final ThroatCounter throat;
    private final RivalNectarDetector camera;   // puede no tener cámara
    private final Random random = new Random();

    private State state = State.IDLE;
    private Command command = Command.STOP;
    private boolean rivalOverride = false;
    private boolean filaAccepting = true;
    private long lastNs = 0;

    private boolean initDone = false;

    // Motor, sin mandar comandos repetidos
    private double motorPower = Double.NaN;
    private long overSinceNs = -1;              // M_F arriba del límite de corriente desde aquí
    private Command stallCommand = Command.STOP;

    // Boca, atascos y rechazo
    private long mouthHeldSinceNs = -1;         // BB1 cortado sin parar desde aquí
    private long mouthSinceNs = -1;             // lo mismo, pero solo jalando (para el atasco)
    private int unjamTries = 0;
    private long unjamUntilNs;
    private long rejectStartNs = -1;            // empieza cuando la lengua ya está abajo
    private String warning = "";

    public IntakeLengua(DcMotorEx motor, Servo servo, BeamSensor mouth, BeamSensor throatBeam,
                        BallSensor throatColor, RivalNectarDetector camera, Alliance alliance) {
        this.motor = motor;
        this.tongue = new TongueServo(servo);
        this.mouth = mouth;
        this.throat = new ThroatCounter(throatBeam, throatColor, alliance);
        this.camera = camera;
        motor.setDirection(IntakeLenguaCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motorPower = 0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // el encoder no se usa
        motor.setCurrentAlert(IntakeLenguaCfg.MF_CURRENT_LIMIT_A, CurrentUnit.AMPS);
    }

    // ===================== INIT =====================

    /**
     * Lengua a plegada y luego suelta (R103). Con una pelota en la boca o en la garganta no
     * se pliega (y así tampoco cabría en R102): avisa y espera. Sin calibrar no se mueve.
     * Regresa true cuando ya quedó plegada.
     */
    @Override
    public boolean initStep(long nowNs) {
        lastNs = nowNs;
        setMotor(0);
        throat.update(nowNs, false, false);
        trackMouth(nowNs);
        if (!IntakeLenguaCfg.CALIBRATED) {
            warning = UNCALIBRATED;
            initDone = false;
            return false;
        }
        if (tongue.target() == null) {
            if (ballUnderFingers()) {
                warning = "pelota en la boca o en la garganta: sácala para plegar la lengua (R102)";
            } else {
                warning = "";
                tongue.fold(nowNs);
            }
        }
        tongue.update(nowNs);
        initDone = tongue.isAt(TongueServo.Position.FOLDED) && !tongue.isPwmOn();
        return initDone;
    }

    @Override
    public boolean isStartReady() {
        return initDone;
    }

    // ===================== partido =====================

    @Override
    public void setCommand(Command command) {
        this.command = command;
    }

    @Override
    public void setRivalOverride(boolean ignoreCamera) {
        rivalOverride = ignoreCamera;
    }

    @Override
    public void update(long nowNs, boolean robotFull, boolean filaAccepting) {
        lastNs = nowNs;
        this.filaAccepting = filaAccepting;
        throat.update(nowNs, state == State.PULLING, state == State.EJECTING);
        trackMouth(nowNs);
        if (!IntakeLenguaCfg.CALIBRATED) {            // seguro: nada se mueve
            setMotor(0);
            state = State.IDLE;
            warning = UNCALIBRATED;
            return;
        }
        // Dieron Play sin pasar por INIT: plegarla, si no hay pelota bajo los dedos
        if (tongue.target() == null && !ballUnderFingers()) tongue.fold(nowNs);
        if (throat.justCounted()) {
            unjamTries = 0;
            mouthSinceNs = -1;
            if (warning.startsWith("atasco")) warning = "";
        }
        checkOverCurrent(nowNs);
        if (throat.newRival() && state != State.REJECTING) {
            warning = "NECTAR rival adentro: sacándola";
            enter(State.REJECTING);
        }
        boolean rivalAhead = camera != null && !rivalOverride && camera.rivalAhead();

        switch (state) {
            case REJECTING: {
                tongue.deploy(nowNs);
                if (!tongue.isAt(TongueServo.Position.DOWN)) {   // primero abajo, con la fila quieta
                    setMotor(0);
                    break;
                }
                if (rejectStartNs < 0) rejectStartNs = nowNs;
                double t = (nowNs - rejectStartNs) / 1e9;
                if (t >= IntakeLenguaCfg.REJECT_S && !throat.rivalNow()) {
                    warning = "";
                    enter(next(robotFull, rivalAhead));
                    runSteady(nowNs);
                } else if (t >= IntakeLenguaCfg.REJECT_MAX_S) {
                    setMotor(0);
                    warning = "NECTAR rival atorada en la entrada: sácala con reversa (bumper izq. 1)";
                    enter(State.RIVAL_STUCK);
                } else {
                    setMotor(-IntakeLenguaCfg.IN_POWER);
                }
                break;
            }

            case RIVAL_STUCK:            // no se jala ni se mueve la fila hasta que se vaya
                if (command == Command.OUT) {
                    enter(State.EJECTING);
                    runSteady(nowNs);
                } else if (!throat.rivalNow()) {
                    warning = "";
                    enter(next(robotFull, rivalAhead));
                    runSteady(nowNs);
                } else {
                    setMotor(0);
                }
                break;

            case STALLED:                // sobrecorriente: quieto hasta que el piloto pida otra cosa (o se llene)
                if (command != stallCommand || robotFull) {
                    warning = "";
                    enter(next(robotFull, rivalAhead));
                    runSteady(nowNs);
                } else {
                    setMotor(0);
                }
                break;

            case UNJAM:
                setMotor(-IntakeLenguaCfg.IN_POWER);
                if (nowNs >= unjamUntilNs) enter(State.PULLING);
                break;

            default: {
                State n = next(robotFull, rivalAhead);
                if (n != state) enter(n);
                runSteady(nowNs);
                break;
            }
        }

        tongue.update(nowNs);
        if (state != State.BLOCKED && warning.startsWith("NECTAR rival enfrente")) warning = "";
    }

    /** Qué estado toca según lo que pide el piloto y lo que pasa. */
    private State next(boolean robotFull, boolean rivalAhead) {
        if (command == Command.OUT) return State.EJECTING;
        if (robotFull) return State.FULL;
        if (command == Command.IN) {
            if (rivalAhead) {
                warning = "NECTAR rival enfrente: no se jala (G408)";
                return State.BLOCKED;
            }
            return State.PULLING;
        }
        return State.IDLE;
    }

    private void runSteady(long nowNs) {
        switch (state) {
            case PULLING:
                tongue.deploy(nowNs);
                if (!tongue.isAt(TongueServo.Position.DOWN)) {
                    setMotor(0);
                    mouthSinceNs = -1;
                    break;
                }
                if (!filaAccepting && (mouth.isActive() || throat.hasBall())) {
                    setMotor(0);                         // espera a que la fila jale (BRAKE): no es atasco
                    mouthSinceNs = -1;
                    break;
                }
                setMotor(IntakeLenguaCfg.IN_POWER);
                if (mouth.isActive()) {                  // atasco en la boca
                    if (mouthSinceNs < 0) mouthSinceNs = nowNs;
                    if ((nowNs - mouthSinceNs) / 1e9 >= IntakeLenguaCfg.UNJAM_MOUTH_S) {
                        mouthSinceNs = -1;
                        if (unjamTries < IntakeLenguaCfg.UNJAM_MAX_TRIES) {
                            unjamTries++;
                            double ms = IntakeLenguaCfg.UNJAM_MIN_MS
                                    + random.nextDouble() * (IntakeLenguaCfg.UNJAM_MAX_MS - IntakeLenguaCfg.UNJAM_MIN_MS);
                            unjamUntilNs = nowNs + (long) (ms * 1e6);
                            enter(State.UNJAM);
                            setMotor(-IntakeLenguaCfg.IN_POWER);
                        } else {
                            warning = "atasco en la boca del intake";
                        }
                    }
                } else {
                    mouthSinceNs = -1;
                }
                break;
            case EJECTING:
                tongue.deploy(nowNs);
                setMotor(tongue.isAt(TongueServo.Position.DOWN) ? -IntakeLenguaCfg.IN_POWER : 0);
                break;
            case FULL:
                setMotor(0);    // BRAKE; la lengua se queda donde está (abajo: subirla empujaría la 4a)
                break;
            case BLOCKED:
            case IDLE:
            default:
                setMotor(0);
                break;
        }
    }

    /**
     * M_F arriba del límite de corriente MF_OVERCURRENT_S seguidos (la marca sale de la
     * lectura en bloque del hub: no cuesta nada): se detiene en BRAKE y avisa. Jalando es un
     * atasco; sacando una rival, la rival se queda atorada (RIVAL_STUCK).
     */
    private void checkOverCurrent(long nowNs) {
        boolean over = motorPower != 0 && motor.isOverCurrent();
        if (!over) {
            overSinceNs = -1;
            return;
        }
        if (overSinceNs < 0) overSinceNs = nowNs;
        if ((nowNs - overSinceNs) / 1e9 < IntakeLenguaCfg.MF_OVERCURRENT_S) return;
        overSinceNs = -1;
        boolean pulling = motorPower > 0;
        setMotor(0);
        if (state == State.REJECTING) {
            warning = "M_F con sobrecorriente sacando la NECTAR rival: sácala con reversa (bumper izq. 1) o a mano";
            enter(State.RIVAL_STUCK);
        } else {
            warning = pulling ? "M_F con sobrecorriente (atasco): suelta y vuelve a mandar el intake"
                    : "M_F con sobrecorriente en reversa: suelta y vuelve a mandar el intake";
            stallCommand = command;
            enter(State.STALLED);
        }
    }

    // ===================== boca =====================

    private void trackMouth(long nowNs) {
        if (!mouth.isActive()) {
            mouthHeldSinceNs = -1;
        } else if (mouthHeldSinceNs < 0) {
            mouthHeldSinceNs = nowNs;
        }
    }

    /** Hay pelota en la boca o en la garganta: los dedos centrales la tocarían al plegar. */
    private boolean ballUnderFingers() {
        return throat.hasBall() || mouth.isActive();
    }

    // ===================== lengua =====================

    /** A dónde se mandó la lengua (null = a ningún lado todavía). */
    public TongueServo.Position getTongue() { return tongue.target(); }

    /** La lengua ya llegó a donde se mandó y está quieta. */
    public boolean isTongueSettled() { return tongue.isSettled(); }

    public boolean isTonguePwmOn() { return tongue.isPwmOn(); }

    // ===================== motor y estado =====================

    private void setMotor(double p) {
        if (p != motorPower) {
            motor.setPower(p);
            motorPower = p;
        }
    }

    private void enter(State s) {
        state = s;
        if (s == State.REJECTING) rejectStartNs = -1;
    }

    @Override
    public boolean isFeeding() {
        return (state == State.PULLING || state == State.UNJAM) && tongue.isAt(TongueServo.Position.DOWN);
    }

    @Override
    public boolean wantsFilaReverse() {
        return (state == State.REJECTING && rejectStartNs >= 0)
                || (state == State.EJECTING && throat.rivalPending() && tongue.isAt(TongueServo.Position.DOWN));
    }

    @Override
    public boolean holdsFila() {
        return state == State.RIVAL_STUCK || (state == State.REJECTING && rejectStartNs < 0)
                || (!IntakeLenguaCfg.CALIBRATED && throat.rivalPending());   // sin calibrar no se puede sacar
    }

    @Override
    public int consumeEntered() {
        return throat.consumeEntered();
    }

    @Override
    public int consumeRejected() {
        return throat.consumeRejected();
    }

    @Override
    public boolean hasBallAtThroat() {
        return throat.hasBall();
    }

    /** En la garganta, o en la boca (BB1) de forma continua FULL_MOUTH_S. */
    @Override
    public boolean hasBallAtEntry() {
        return throat.hasBall()
                || (mouthHeldSinceNs >= 0 && (lastNs - mouthHeldSinceNs) / 1e9 >= IntakeLenguaCfg.FULL_MOUTH_S);
    }

    @Override
    public BallType lastBallType() {
        return throat.lastType();
    }

    @Override
    public String warning() {
        return warning;
    }

    @Override
    public void stop() {
        setMotor(0);
        // La lengua y la cámara se quedan: el SDK apaga el PWM y cierra el VisionPortal al parar.
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake lengua", "%s  lengua %s  boca %s  garganta %s  color %s",
                state, tongue.describe(),
                mouth.isActive() ? "sí" : "no", throat.hasBall() ? "sí" : "no",
                throat.colorFault() ? "SIN LECTURA" : throat.colorLast());
        if (camera != null) telemetry.addData("  cámara", camera.describe());
        if (!warning.isEmpty()) telemetry.addLine("!! " + warning);
    }
}
