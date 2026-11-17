package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config.IntakeF2Cfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.vision.RivalNectarDetector;

import java.util.Random;

/**
 * Intake F2 del CAD (v49 en adelante): rotor con 12 módulos de aletas de TPU movido por
 * M_F (Yellow Jacket 5203-2402-0005) con banda 20:30. Su encoder da 217.65 cuentas por
 * vuelta del rotor (1.65° por cuenta).
 *
 * Sensores: break-beam BB1 en la boca, break-beam de la garganta (cuenta pelotas),
 * REV Color Sensor V3 a la entrada de la fila (tipo y alianza), REV Magnetic Limit Switch
 * REV-31-1462 (índice para estacionar) y la cámara C270 de la visera.
 *
 * Lo que pide la especificación (intake_modular/spec_intake_modular.md):
 *  - INIT (R103.B): gira despacio hasta el flanco del sensor magnético, primero en un
 *    sentido y luego en el otro; el índice es el promedio de los dos flancos. Va a 0° y
 *    se queda en BRAKE. Tolerancia ±5° (con más, el robot no cabe en R102).
 *  - Jalar al 100 %.
 *  - Atasco: si BB1 ve pelota más de 250 ms y la garganta no cuenta ninguna, reversa de
 *    60 a 100 ms (al azar) y vuelve a jalar, hasta 3 veces; luego avisa.
 *  - Con 4 pelotas (G407): modo lleno = va a +25° y frena en BRAKE (no reversa).
 *  - G408: si la cámara ve una NECTAR rival enfrente, no jala y avisa. Si el V3 la ve
 *    adentro, reversa del intake y de la fila de 0.3 a 0.5 s.
 *
 * A Adrián no le convence F2 y se va a cambiar: esta clase vive detrás de Intake.
 */
public class IntakeF2 implements Intake {
    private enum State {
        WAITING_PARK, PARK_CLEAR, PARK_FWD, PARK_PASS, PARK_BACK, PARK_GOTO, PARKED, PARK_FAILED,
        IDLE, PULLING, UNJAM, BLOCKED, REJECTING, EJECTING, FULL_GOTO, FULL_HOLD
    }

    private enum Drive { NONE, POWER, VELOCITY, TO_POSITION }

    private final DcMotorEx motor;
    private final BeamSensor magnet, mouth, throat;
    private final BallSensor throatColor;
    private final RivalNectarDetector camera;   // puede no tener cámara
    private final Alliance alliance;
    private final Random random = new Random();

    private State state = State.WAITING_PARK;
    private long stateStartNs;
    private Command command = Command.STOP;
    private boolean rivalOverride = false;

    // Índice del rotor (cuentas del encoder en 0°)
    private boolean indexKnown = false;
    private double indexTicks;
    private double edgeFwd, clearFromTicks = Double.NaN, searchFromTicks;
    private double inactiveFromTicks = Double.NaN;   // desde dónde se ve el imán ausente
    private long withinSinceNs = -1;                  // desde cuándo está cerca del objetivo

    // Control del motor sin mandar comandos repetidos
    private Drive drive = Drive.NONE;
    private double driveValue = Double.NaN;

    // Conteo y atascos
    private boolean throatWasActive = false;
    private boolean throatCounted = false;
    private long throatSinceNs;
    private long mouthSinceNs = -1;
    private int unjamTries = 0;
    private long unjamUntilNs, rejectUntilNs;
    private int entered = 0, rejected = 0;
    private BallType lastType = BallType.NONE;
    private boolean rivalWasSeen = false;
    private String warning = "";

    public IntakeF2(DcMotorEx motor, BeamSensor magnet, BeamSensor mouth, BeamSensor throat,
                    BallSensor throatColor, RivalNectarDetector camera, Alliance alliance) {
        this.motor = motor;
        this.magnet = magnet;
        this.mouth = mouth;
        this.throat = throat;
        this.throatColor = throatColor;
        this.camera = camera;
        this.alliance = alliance;
        motor.setDirection(IntakeF2Cfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPositionTolerance(IntakeF2Cfg.PARK_TOLERANCE_TICKS);
        drivePower(0);
    }

    // ===================== INIT: estacionar =====================

    @Override
    public boolean initStep(long nowNs) {
        double pos = motor.getCurrentPosition();
        double clearTicks = IntakeF2Cfg.PARK_CLEAR_DEG * ticksPerDeg();
        double rev = IntakeF2Cfg.TICKS_PER_ROTOR_REV;
        boolean timeout = secondsIn(nowNs) > IntakeF2Cfg.PARK_STEP_TIMEOUT_S;

        switch (state) {
            case WAITING_PARK:
                clearFromTicks = Double.NaN;
                inactiveFromTicks = Double.NaN;
                searchFromTicks = pos;
                enter(magnet.isActive() ? State.PARK_CLEAR : State.PARK_FWD, nowNs);
                return false;

            case PARK_CLEAR:          // arrancó sobre el imán: salir hacia atrás
                driveVelocity(-IntakeF2Cfg.PARK_SEARCH_TICKS_PER_S);
                if (!magnet.isActive()) {
                    if (Double.isNaN(clearFromTicks)) clearFromTicks = pos;
                    if (clearFromTicks - pos >= clearTicks) {
                        searchFromTicks = pos;
                        inactiveFromTicks = pos;
                        enter(State.PARK_FWD, nowNs);
                    }
                }
                if (timeout) failPark(nowNs, "no salió del imán");
                return false;

            case PARK_FWD:            // primer flanco, girando hacia adelante
                driveVelocity(IntakeF2Cfg.PARK_SEARCH_TICKS_PER_S);
                if (!magnet.isActive()) {
                    if (Double.isNaN(inactiveFromTicks)) inactiveFromTicks = pos;
                } else {
                    // Solo es flanco si antes se vio el imán ausente un tramo: una lectura
                    // vieja al arrancar sobre el imán no debe contar como entrada.
                    boolean armed = !Double.isNaN(inactiveFromTicks)
                            && pos - inactiveFromTicks >= IntakeF2Cfg.PARK_ARM_DEG * ticksPerDeg();
                    inactiveFromTicks = Double.NaN;
                    if (armed) {
                        edgeFwd = pos;
                        clearFromTicks = Double.NaN;
                        enter(State.PARK_PASS, nowNs);
                    }
                }
                if (state == State.PARK_FWD && (pos - searchFromTicks > 1.3 * rev || timeout)) {
                    failPark(nowNs, "no encontró el imán");
                }
                return false;

            case PARK_PASS:           // pasarse del imán para regresar
                driveVelocity(IntakeF2Cfg.PARK_SEARCH_TICKS_PER_S);
                if (!magnet.isActive()) {
                    if (Double.isNaN(clearFromTicks)) clearFromTicks = pos;
                    if (pos - clearFromTicks >= clearTicks) {
                        searchFromTicks = pos;
                        enter(State.PARK_BACK, nowNs);
                    }
                }
                if (timeout) failPark(nowNs, "se quedó sobre el imán");
                return false;

            case PARK_BACK:           // segundo flanco, girando hacia atrás
                driveVelocity(-IntakeF2Cfg.PARK_SEARCH_TICKS_PER_S);
                if (magnet.isActive()) {
                    indexTicks = 0.5 * (edgeFwd + pos) + IntakeF2Cfg.INDEX_OFFSET_DEG * ticksPerDeg();
                    indexKnown = true;
                    enter(State.PARK_GOTO, nowNs);
                } else if (searchFromTicks - pos > 0.6 * rev || timeout) {
                    failPark(nowNs, "perdió el imán de regreso");
                }
                return false;

            case PARK_GOTO: {
                double target = nearest(indexTicks, pos);
                driveTo(target);
                if (arrived(pos, target, nowNs) || timeout) {
                    drivePower(0);
                    enter(State.PARKED, nowNs);
                    return true;
                }
                return false;
            }

            case PARKED:
            case PARK_FAILED:
            default:
                return true;
        }
    }

    private void failPark(long nowNs, String why) {
        drivePower(0);
        warning = "F2 sin estacionar: " + why;
        enter(State.PARK_FAILED, nowNs);
    }

    /** Error de estacionado en grados (para "Tune: Intake F2" y la inspección). */
    public double parkErrorDeg() {
        if (!indexKnown) return Double.NaN;
        double pos = motor.getCurrentPosition();
        return (pos - nearest(indexTicks, pos)) / ticksPerDeg();
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
        if (isParkingState(state)) {          // dieron Play antes de terminar: se deja como esté
            drivePower(0);
            enter(State.IDLE, nowNs);
        }

        boolean mouthBall = mouth.isActive();
        boolean throatBall = throat.isActive();
        BallType seen = throatColor.read(nowNs);
        boolean pullingIn = state == State.PULLING;

        // Conteo con el break-beam de la garganta.
        //  - Hacia adentro se cuenta al LLEGAR (haz cortado ≥ THROAT_MIN_S): la 4a pelota se
        //    queda parada sobre el haz y nunca lo libera; contarla al salir la contaría dos
        //    veces cuando después avance a la fila.
        //  - Hacia afuera (reversa del piloto) se cuenta al LIBERAR: ya salió.
        if (throatBall && !throatWasActive) {
            throatSinceNs = nowNs;
            throatCounted = false;
        }
        if (throatBall && !throatCounted && pullingIn
                && (nowNs - throatSinceNs) / 1e9 >= IntakeF2Cfg.THROAT_MIN_S) {
            throatCounted = true;
            entered++;
            unjamTries = 0;
            mouthSinceNs = -1;
            if (warning.startsWith("atasco")) warning = "";
        }
        if (!throatBall && throatWasActive && state == State.EJECTING
                && (nowNs - throatSinceNs) / 1e9 >= IntakeF2Cfg.THROAT_MIN_S) {
            rejected++;
        }
        throatWasActive = throatBall;
        if (seen.isBall()) lastType = seen;

        // G408 adentro: NECTAR rival en la entrada de la fila (una vez por pelota)
        boolean rivalNow = seen.isRivalFor(alliance);
        boolean rivalNew = rivalNow && !rivalWasSeen;
        rivalWasSeen = rivalNow;
        if (rivalNew && state != State.REJECTING) {
            rejected++;
            rejectUntilNs = nowNs + (long) (IntakeF2Cfg.REJECT_S * 1e9);
            warning = "NECTAR rival adentro: sacándola";
            enter(State.REJECTING, nowNs);
        }

        boolean rivalAhead = camera != null && !rivalOverride && camera.rivalAhead();

        switch (state) {
            case REJECTING:
                drivePower(-IntakeF2Cfg.IN_POWER);
                if (nowNs >= rejectUntilNs) enter(nextFromCommand(robotFull, rivalAhead), nowNs);
                break;

            case UNJAM:
                drivePower(-IntakeF2Cfg.IN_POWER);
                if (nowNs >= unjamUntilNs) enter(State.PULLING, nowNs);
                break;

            case FULL_GOTO:
                if (!robotFull) {
                    enter(nextFromCommand(false, rivalAhead), nowNs);
                    break;
                }
                double pos = motor.getCurrentPosition();
                double target = fullTarget(pos);
                driveTo(target);
                if (arrived(pos, target, nowNs) || secondsIn(nowNs) > IntakeF2Cfg.FULL_GOTO_TIMEOUT_S) {
                    drivePower(0);
                    enter(State.FULL_HOLD, nowNs);
                }
                break;

            case FULL_HOLD:
                drivePower(0);
                if (!robotFull) enter(nextFromCommand(false, rivalAhead), nowNs);
                else if (command == Command.OUT) enter(State.EJECTING, nowNs);
                break;

            default:
                State next = nextFromCommand(robotFull, rivalAhead);
                if (next != state) enter(next, nowNs);
                runSteady(nowNs, mouthBall);
                break;
        }

        if (state != State.BLOCKED && warning.startsWith("NECTAR rival enfrente")) warning = "";
    }

    /** Qué estado toca según lo que pide el piloto y lo que pasa. */
    private State nextFromCommand(boolean robotFull, boolean rivalAhead) {
        if (command == Command.OUT) return State.EJECTING;
        if (robotFull) return indexKnown ? State.FULL_GOTO : State.FULL_HOLD;
        if (command == Command.IN) {
            if (rivalAhead) {
                warning = "NECTAR rival enfrente: no se jala (G408)";
                return State.BLOCKED;
            }
            return State.PULLING;
        }
        return State.IDLE;
    }

    private void runSteady(long nowNs, boolean mouthBall) {
        switch (state) {
            case PULLING:
                drivePower(IntakeF2Cfg.IN_POWER);
                // Atasco en la boca
                if (mouthBall) {
                    if (mouthSinceNs < 0) mouthSinceNs = nowNs;
                    if ((nowNs - mouthSinceNs) / 1e9 >= IntakeF2Cfg.UNJAM_MOUTH_S) {
                        mouthSinceNs = -1;
                        if (unjamTries < IntakeF2Cfg.UNJAM_MAX_TRIES) {
                            unjamTries++;
                            double ms = IntakeF2Cfg.UNJAM_MIN_MS
                                    + random.nextDouble() * (IntakeF2Cfg.UNJAM_MAX_MS - IntakeF2Cfg.UNJAM_MIN_MS);
                            unjamUntilNs = nowNs + (long) (ms * 1e6);
                            enter(State.UNJAM, nowNs);
                        } else {
                            warning = "atasco en la boca del intake";
                        }
                    }
                } else {
                    mouthSinceNs = -1;
                }
                break;
            case EJECTING:
                drivePower(-IntakeF2Cfg.IN_POWER);
                break;
            case BLOCKED:
            case IDLE:
            default:
                drivePower(0);
                break;
        }
    }

    /** Llegó si lleva GOTO_SETTLE_S dentro de GOTO_ACCEPT_TICKS del objetivo. */
    private boolean arrived(double pos, double target, long nowNs) {
        if (Math.abs(pos - target) > IntakeF2Cfg.GOTO_ACCEPT_TICKS) {
            withinSinceNs = -1;
            return false;
        }
        if (withinSinceNs < 0) withinSinceNs = nowNs;
        return (nowNs - withinSinceNs) / 1e9 >= IntakeF2Cfg.GOTO_SETTLE_S;
    }

    /** +25° del índice, la próxima vez que pase girando hacia adelante. */
    private double fullTarget(double pos) {
        double rev = IntakeF2Cfg.TICKS_PER_ROTOR_REV;
        double base = indexTicks + IntakeF2Cfg.FULL_HOLD_DEG * ticksPerDeg();
        double k = Math.ceil((pos - base - IntakeF2Cfg.PARK_TOLERANCE_TICKS) / rev);
        return base + k * rev;
    }

    /** La posición equivalente a target (módulo una vuelta) más cercana a pos. */
    private static double nearest(double target, double pos) {
        double rev = IntakeF2Cfg.TICKS_PER_ROTOR_REV;
        return target + Math.round((pos - target) / rev) * rev;
    }

    private static double ticksPerDeg() {
        return IntakeF2Cfg.TICKS_PER_ROTOR_REV / 360.0;
    }

    private static boolean isParkingState(State s) {
        return s == State.WAITING_PARK || s == State.PARK_CLEAR || s == State.PARK_FWD
                || s == State.PARK_PASS || s == State.PARK_BACK || s == State.PARK_GOTO;
    }

    // ===================== motor =====================

    private void drivePower(double p) {
        if (drive != Drive.POWER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive = Drive.POWER;
            driveValue = Double.NaN;
        }
        if (p != driveValue) {
            motor.setPower(p);
            driveValue = p;
        }
    }

    private void driveVelocity(double ticksPerS) {
        if (drive != Drive.VELOCITY) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive = Drive.VELOCITY;
            driveValue = Double.NaN;
        }
        if (ticksPerS != driveValue) {
            motor.setVelocity(ticksPerS);
            driveValue = ticksPerS;
        }
    }

    private void driveTo(double ticks) {
        int t = (int) Math.round(ticks);
        if (drive != Drive.TO_POSITION || t != (int) driveValue) {
            motor.setTargetPosition(t);           // primero el objetivo, luego el modo
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(IntakeF2Cfg.GOTO_POWER);
            drive = Drive.TO_POSITION;
            driveValue = t;
        }
    }

    // ===================== estado =====================

    private void enter(State s, long nowNs) {
        state = s;
        stateStartNs = nowNs;
        withinSinceNs = -1;
    }

    private double secondsIn(long nowNs) {
        return (nowNs - stateStartNs) / 1e9;
    }

    @Override
    public boolean isFeeding() {
        return state == State.PULLING || state == State.UNJAM;
    }

    @Override
    public boolean wantsFilaReverse() {
        return state == State.REJECTING;
    }

    @Override
    public int consumeEntered() {
        int n = entered;
        entered = 0;
        return n;
    }

    @Override
    public int consumeRejected() {
        int n = rejected;
        rejected = 0;
        return n;
    }

    @Override
    public boolean hasBallAtThroat() {
        return throat.isActive();
    }

    @Override
    public BallType lastBallType() {
        return lastType;
    }

    @Override
    public String warning() {
        return warning;
    }

    public boolean isParked() {
        return state == State.PARKED;
    }

    public boolean isIndexKnown() {
        return indexKnown;
    }

    @Override
    public void stop() {
        drivePower(0);
        if (camera != null) camera.close();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake F2", "%s  boca %s  garganta %s  color %s",
                state, mouth.isActive() ? "sí" : "no", throat.isActive() ? "sí" : "no", throatColor.last());
        if (camera != null) telemetry.addData("  cámara", camera.describe());
        if (!warning.isEmpty()) telemetry.addLine("!! " + warning);
    }
}
