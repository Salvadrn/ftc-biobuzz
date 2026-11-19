package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.vision.RivalNectarDetector;

import java.util.Random;

/**
 * Intake de la lengua de bandas con dedos colgantes (CAD, 18-nov; todavía no modelada).
 * M_F mueve las bandas y los 4 dedos; un servo 2000-0025-0002 con manivela y biela
 * despliega la lengua. No hay estacionado: ni imán ni encoder.
 *
 * La lengua tiene dos posiciones y no tiene sensor: "ya llegó" = TRAVEL_S después de mandarla.
 *  - INIT: a plegada y luego PWM apagado; se sostiene sola contra el tope de atrás (R103) y
 *    el robot queda quieto (G304.H).
 *  - Solo se pliega con la garganta LIBRE: con una pelota ahí (la 4a), los dedos centrales
 *    se montan en ella y, al subir, la empujarían contra la flecha del pivote de la fila
 *    (CAD v106: desde ~12° con NECTAR y ~25° con POLLEN). Sería controlarla y podría romper.
 *  - Al dar Play se queda plegada. Baja cuando el intake está prendido y hay menos de 4
 *    pelotas. Propuesta del CAD; Adrián decide (y la precarga).
 *  - Con 4 pelotas (G407) se queda ABAJO y M_F para en BRAKE (CAD v106, por omisión).
 *  - Abajo, PWM prendido (por ahora, CAD). M_F solo jala con la lengua abajo.
 *
 * Jalar al 100 %, atasco por BB1, cuenta en la garganta (ThroatCounter) y G408 con la
 * cámara y el V3 de la garganta. Para regresar una rival la lengua tiene que estar ABAJO:
 * si estaba plegada, primero baja, con la fila quieta, y luego reversa de M_F y de la fila.
 */
public class IntakeLengua implements Intake {
    /** Posiciones de la lengua. */
    public enum Tongue { FOLDED, DOWN }

    private enum State { IDLE, PULLING, UNJAM, BLOCKED, REJECTING, RIVAL_STUCK, EJECTING, FULL }

    /** 2000-0025-0002: 500 a 2500 µs. */
    private static final double SERVO_MIN_US = 500, SERVO_MAX_US = 2500;

    private final DcMotor motor;
    private final Servo servo;
    private final BeamSensor mouth;
    private final ThroatCounter throat;
    private final RivalNectarDetector camera;   // puede no tener cámara
    private final Random random = new Random();

    private State state = State.IDLE;
    private Command command = Command.STOP;
    private boolean rivalOverride = false;
    private long lastNs = 0;

    // Lengua
    private Tongue tongue = null;               // a dónde se mandó; null = todavía a ningún lado
    private long tongueCmdNs;
    private boolean tonguePwmOn = true;
    private boolean initDone = false;

    // Motor, sin mandar comandos repetidos
    private double motorPower = Double.NaN;

    // Atascos y rechazo
    private long mouthSinceNs = -1;
    private int unjamTries = 0;
    private long unjamUntilNs;
    private long rejectStartNs = -1;            // empieza cuando la lengua ya está abajo
    private String warning = "";

    public IntakeLengua(DcMotor motor, Servo servo, BeamSensor mouth, BeamSensor throatBeam,
                        BallSensor throatColor, RivalNectarDetector camera, Alliance alliance) {
        this.motor = motor;
        this.servo = servo;
        this.mouth = mouth;
        this.throat = new ThroatCounter(throatBeam, throatColor, alliance);
        this.camera = camera;
        motor.setDirection(IntakeLenguaCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motorPower = 0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // el encoder ya no se usa
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(SERVO_MIN_US, SERVO_MAX_US));
        }
    }

    // ===================== INIT =====================

    /**
     * Lengua a plegada y luego suelta (R103). Con una pelota en la garganta no se pliega
     * (y así tampoco cabría en R102): avisa y espera. Regresa true cuando ya quedó plegada.
     */
    @Override
    public boolean initStep(long nowNs) {
        lastNs = nowNs;
        setMotor(0);
        throat.update(nowNs, false, false);
        if (tongue == null) {
            if (throat.hasBall()) {
                warning = "pelota en la garganta: sácala para plegar la lengua (R102)";
            } else {
                warning = "";
                moveTongue(Tongue.FOLDED, nowNs);
            }
        }
        updateTongue(nowNs);
        initDone = tongue == Tongue.FOLDED && !tonguePwmOn;
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
    public void update(long nowNs, boolean robotFull) {
        lastNs = nowNs;
        throat.update(nowNs, state == State.PULLING, state == State.EJECTING);
        // Dieron Play sin pasar por INIT: plegarla, si la garganta está libre
        if (tongue == null && !throat.hasBall()) moveTongue(Tongue.FOLDED, nowNs);
        if (throat.justCounted()) {
            unjamTries = 0;
            mouthSinceNs = -1;
            if (warning.startsWith("atasco")) warning = "";
        }
        if (throat.newRival() && state != State.REJECTING) {
            warning = "NECTAR rival adentro: sacándola";
            enter(State.REJECTING);
        }
        boolean rivalAhead = camera != null && !rivalOverride && camera.rivalAhead();

        switch (state) {
            case REJECTING: {
                moveTongue(Tongue.DOWN, nowNs);
                if (!tongueAt(Tongue.DOWN, nowNs)) {     // primero abajo, con la fila quieta
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

        updateTongue(nowNs);
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
                moveTongue(Tongue.DOWN, nowNs);
                if (!tongueAt(Tongue.DOWN, nowNs)) {
                    setMotor(0);
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
                moveTongue(Tongue.DOWN, nowNs);
                setMotor(tongueAt(Tongue.DOWN, nowNs) ? -IntakeLenguaCfg.IN_POWER : 0);
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

    // ===================== lengua =====================

    private void moveTongue(Tongue t, long nowNs) {
        if (t == tongue) return;
        tongue = t;
        tongueCmdNs = nowNs;
        if (servo instanceof PwmControl) ((PwmControl) servo).setPwmEnable();
        tonguePwmOn = true;
        servo.setPosition(positionFor(t));
    }

    /** Plegada y ya llegó: se suelta (se sostiene sola por su peso). */
    private void updateTongue(long nowNs) {
        if (tongue == Tongue.FOLDED && tonguePwmOn && tongueAt(Tongue.FOLDED, nowNs)) {
            if (servo instanceof PwmControl) ((PwmControl) servo).setPwmDisable();
            tonguePwmOn = false;
        }
    }

    private boolean tongueAt(Tongue t, long nowNs) {
        return tongue == t && (nowNs - tongueCmdNs) / 1e9 >= IntakeLenguaCfg.TRAVEL_S;
    }

    public static double usFor(Tongue t) {
        return t == Tongue.DOWN ? IntakeLenguaCfg.DOWN_US : IntakeLenguaCfg.FOLDED_US;
    }

    public static double positionFor(Tongue t) {
        return Range.clip((usFor(t) - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US), 0, 1);
    }

    /** A dónde se mandó la lengua (null = a ningún lado todavía). */
    public Tongue getTongue() { return tongue; }

    /** La lengua ya tuvo tiempo de llegar a donde se mandó. */
    public boolean isTongueSettled() { return tongue != null && tongueAt(tongue, lastNs); }

    public boolean isTonguePwmOn() { return tonguePwmOn; }

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
        return (state == State.PULLING || state == State.UNJAM) && tongueAt(Tongue.DOWN, lastNs);
    }

    @Override
    public boolean wantsFilaReverse() {
        return (state == State.REJECTING && rejectStartNs >= 0)
                || (state == State.EJECTING && throat.rivalPending() && tongueAt(Tongue.DOWN, lastNs));
    }

    @Override
    public boolean holdsFila() {
        return state == State.RIVAL_STUCK || (state == State.REJECTING && rejectStartNs < 0);
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
        telemetry.addData("Intake lengua", "%s  lengua %s%s  boca %s  garganta %s  color %s",
                state, tongue, isTongueSettled() ? "" : " (moviéndose)",
                mouth.isActive() ? "sí" : "no", throat.hasBall() ? "sí" : "no",
                throat.colorFault() ? "SIN LECTURA" : throat.colorLast());
        if (camera != null) telemetry.addData("  cámara", camera.describe());
        if (IntakeLenguaCfg.FOLDED_US == IntakeLenguaCfg.DOWN_US) {
            telemetry.addLine("!! Lengua sin calibrar: faltan los µs de plegada y abajo (CAD)");
        }
        if (!warning.isEmpty()) telemetry.addLine("!! " + warning);
    }
}
