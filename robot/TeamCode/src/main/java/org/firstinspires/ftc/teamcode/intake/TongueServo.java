package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;

/**
 * Servo 2000-0025-0002 que pliega y despliega la lengua del intake con manivela y biela
 * (CAD v107-v118; secuencia del plan de la lengua, plan_lengua_final.json, A/decisiones[11]).
 * Los µs son de cálculo hasta pasar por "Tune: Lengua" (IntakeLenguaCfg.CALIBRATED).
 *
 * El servo no dice dónde está: se estima suponiendo que va a SERVO_DEG_S (pesimista).
 *
 *  - Plegar: directo a plegada. La manivela sube la lengua con la biela tensa; pasada la
 *    vertical, la lengua se va sola unos 5° hasta su tope y ahí descansa por su peso, sin
 *    carga en el servo: entonces se apaga el PWM (nunca se deja suelta en una posición
 *    intermedia: entre plegada y el empuje la lengua queda libre).
 *  - Desplegar, en dos tramos:
 *     1. rápido de plegada a "empuje": en esos 77° de manivela la lengua no se mueve (el
 *        perno corre en la ranura de la biela);
 *     2. de empuje a abajo en rampa, a DEPLOY_RAMP_DEG_S como máximo: la biela lleva la
 *        lengua a la vertical y, al pasarla, la lengua cae por su peso. Con la rampa la
 *        manivela va bajando y la lengua la alcanza sin golpe (quieta, serían ~35° de caída
 *        libre).
 *  - Abajo, PWM prendido.
 *
 * Se llama a update() en cada ciclo; fold()/deploy() solo cambian el objetivo.
 */
public class TongueServo {
    public enum Position { FOLDED, DOWN }

    private enum Phase { STILL, TO_PUSH, RAMP }

    /** 2000-0025-0002: 500 a 2500 µs, 0.15° por µs. */
    public static final double MIN_US = 500, MAX_US = 2500, DEG_PER_US = 0.15;

    private final Servo servo;
    private Position target = null;          // null = todavía a ningún lado (o a mano)
    private boolean manual = false;          // "Tune: Lengua" la está moviendo a mano
    private Phase phase = Phase.STILL;
    private double cmdUs = Double.NaN;        // lo que se le manda
    private double estUs = Double.NaN;        // dónde se cree que va
    private double rampEndUs = Double.NaN;    // dónde termina la rampa (DOWN_US, o antes en "Tune: Lengua")
    private boolean pwmOn = true;
    private long lastNs = 0;
    private long stillSinceNs = 0;

    public TongueServo(Servo servo) {
        this.servo = servo;
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(MIN_US, MAX_US));
        }
    }

    /** Directo a plegada. Sin saber dónde estaba, supone lo peor (abajo) para el tiempo. */
    public void fold(long nowNs) {
        if (target == Position.FOLDED) return;
        if (Double.isNaN(estUs)) estUs = IntakeLenguaCfg.DOWN_US;
        manual = false;
        target = Position.FOLDED;
        phase = Phase.STILL;
        command(IntakeLenguaCfg.FOLDED_US, nowNs);
    }

    /** Rápido hasta el empuje y de ahí en rampa hasta abajo. */
    public void deploy(long nowNs) {
        if (target == Position.DOWN) return;
        startDeploy(IntakeLenguaCfg.DOWN_US, nowNs);
        manual = false;
        target = Position.DOWN;
    }

    /**
     * Solo para "Tune: Lengua": la misma secuencia de desplegar, pero la rampa termina en
     * endUs (la primera vez, 100 µs antes de DOWN_US, para buscar el despegue).
     */
    public void deployForTuning(double endUs, long nowNs) {
        startDeploy(endUs, nowNs);
        target = null;
        manual = true;
    }

    private void startDeploy(double endUs, long nowNs) {
        if (Double.isNaN(estUs)) estUs = IntakeLenguaCfg.FOLDED_US;   // al empezar está plegada (R102)
        rampEndUs = endUs;
        phase = Phase.TO_PUSH;
        command(IntakeLenguaCfg.PUSH_US, nowNs);
    }

    /** Llamar una vez por ciclo: rampa, estimado y PWM. */
    public void update(long nowNs) {
        double dt = lastNs == 0 ? 0 : Range.clip((nowNs - lastNs) / 1e9, 0, 0.1);
        lastNs = nowNs;
        if (target == null && !manual) return;

        double servoStep = IntakeLenguaCfg.SERVO_DEG_S / DEG_PER_US * dt;
        estUs += Range.clip(cmdUs - estUs, -servoStep, servoStep);
        if (Math.abs(estUs - cmdUs) > 0.5) stillSinceNs = nowNs;

        if (phase == Phase.TO_PUSH && Math.abs(estUs - cmdUs) <= 0.5) phase = Phase.RAMP;
        if (phase == Phase.RAMP) {
            double rampStep = IntakeLenguaCfg.DEPLOY_RAMP_DEG_S / DEG_PER_US * dt;
            double next = cmdUs + Range.clip(rampEndUs - cmdUs, -rampStep, rampStep);
            if (next == rampEndUs) phase = Phase.STILL;
            command(next, nowNs);
        }

        if (target == Position.FOLDED && pwmOn && isSettled(nowNs)) {   // descansa por su peso
            if (servo instanceof PwmControl) ((PwmControl) servo).setPwmDisable();
            pwmOn = false;
        }
    }

    /**
     * Solo para "Tune: Lengua": manda un pulso directo, sin secuencia. Usarlo en pasos chicos
     * desde donde ya está: un salto grande desde plegada hacia abajo deja caer la lengua.
     */
    public void manual(double us, long nowNs) {
        if (Double.isNaN(estUs)) estUs = us;
        target = null;
        manual = true;
        phase = Phase.STILL;
        command(us, nowNs);
    }

    /** Solo para "Tune: Lengua": suelta el servo (PWM apagado). */
    public void release() {
        if (servo instanceof PwmControl) ((PwmControl) servo).setPwmDisable();
        pwmOn = false;
    }

    private void command(double us, long nowNs) {
        if (us == cmdUs && pwmOn) return;
        cmdUs = us;
        if (!pwmOn && servo instanceof PwmControl) ((PwmControl) servo).setPwmEnable();
        pwmOn = true;
        servo.setPosition(positionFor(us));
        stillSinceNs = nowNs;
    }

    private boolean isSettled(long nowNs) {
        return phase == Phase.STILL && Math.abs(estUs - cmdUs) <= 0.5
                && (nowNs - stillSinceNs) / 1e9 >= IntakeLenguaCfg.SETTLE_S;
    }

    /** Ya está en p y quieta. */
    public boolean isAt(Position p) {
        return target == p && isSettled(lastNs);
    }

    /** A dónde se mandó (null = a ningún lado todavía). */
    public Position target() { return target; }
    public boolean isSettled() { return (target != null || manual) && isSettled(lastNs); }
    public boolean isPwmOn() { return pwmOn; }
    public double commandUs() { return cmdUs; }
    public double estimateUs() { return estUs; }

    /** Posición 0..1 del servo para un pulso en µs. */
    public static double positionFor(double us) {
        return Range.clip((us - MIN_US) / (MAX_US - MIN_US), 0, 1);
    }

    public String describe() {
        String what = phase == Phase.TO_PUSH ? "yendo al empuje" : phase == Phase.RAMP ? "bajando en rampa"
                : isSettled() ? "quieta" : "moviéndose";
        if (manual) return String.format("a mano, %s (%.0f µs%s)", what, cmdUs, pwmOn ? "" : ", suelta");
        if (target == null) return "sin mandar";
        return String.format("%s %s (%.0f µs%s)", target, what, cmdUs, pwmOn ? "" : ", suelta");
    }
}
