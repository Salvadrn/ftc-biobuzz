import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Proxy;
import java.util.function.BooleanSupplier;

/**
 * Hardware falso para la simulación, con proxies de Java: implementa las interfaces del
 * SDK y guarda lo que el código le manda. La física la avanza cada prueba.
 */
final class Fakes {
    private Fakes() {}

    private static Object defaultFor(Class<?> t) {
        if (t == boolean.class) return false;
        if (t == int.class) return 0;
        if (t == long.class) return 0L;
        if (t == double.class) return 0.0;
        if (t == float.class) return 0f;
        if (t == String.class) return "fake";
        return null;
    }

    // ---------------- motor ----------------
    static final class Motor {
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotor.ZeroPowerBehavior zpb = DcMotor.ZeroPowerBehavior.FLOAT;
        double power, velocityCmd;   // velocityCmd en cuentas/s
        int target, tolerance = 10;
        double pos, vel;             // estado físico (cuentas, cuentas/s)
        int modeChanges = 0;
        boolean targetSetBeforeRunToPosition = true;

        DcMotorEx proxy() {
            return (DcMotorEx) Proxy.newProxyInstance(Fakes.class.getClassLoader(), new Class[]{DcMotorEx.class},
                    (p, m, a) -> {
                        switch (m.getName()) {
                            case "setMode":
                                if (a[0] == DcMotor.RunMode.RUN_TO_POSITION && targetUnset) targetSetBeforeRunToPosition = false;
                                if (mode != a[0]) modeChanges++;
                                mode = (DcMotor.RunMode) a[0];
                                return null;
                            case "getMode": return mode;
                            case "setPower": power = (Double) a[0]; return null;
                            case "getPower": return power;
                            case "setVelocity": velocityCmd = (Double) a[0]; return null;
                            case "getVelocity": return vel;
                            case "setTargetPosition": target = (Integer) a[0]; targetUnset = false; return null;
                            case "getTargetPosition": return target;
                            case "getCurrentPosition": return (int) Math.round(pos);
                            case "setTargetPositionTolerance": tolerance = (Integer) a[0]; return null;
                            case "isBusy": return Math.abs(pos - target) > tolerance;
                            case "setZeroPowerBehavior": zpb = (DcMotor.ZeroPowerBehavior) a[0]; return null;
                            case "getZeroPowerBehavior": return zpb;
                            case "toString": return "FakeMotor";
                            case "hashCode": return System.identityHashCode(p);
                            case "equals": return p == a[0];
                            default: return defaultFor(m.getReturnType());
                        }
                    });
        }
        private boolean targetUnset = true;

        /**
         * Avanza la física un paso. maxTicksPerS = velocidad a potencia 1;
         * en RUN_TO_POSITION va hacia el objetivo con la potencia dada.
         */
        void step(double dt, double maxTicksPerS) {
            switch (mode) {
                case RUN_USING_ENCODER:
                    vel = Math.max(-maxTicksPerS, Math.min(maxTicksPerS, velocityCmd));
                    break;
                case RUN_TO_POSITION: {
                    double err = target - pos;
                    double v = Math.abs(power) * maxTicksPerS;
                    double step = Math.min(Math.abs(err), v * dt);
                    vel = Math.signum(err) * (dt > 0 ? step / dt : 0);
                    break;
                }
                default:
                    vel = power * maxTicksPerS;
            }
            pos += vel * dt;
        }
    }

    // ---------------- servo de posición ----------------
    /**
     * Imita al LynxServoController del SDK 12.0 (leído con javap): un comando de posición
     * DISTINTO al último manda el pulso y vuelve a encender el PWM; uno igual no manda
     * nada; apagar el PWM borra la caché, así que el siguiente comando, aunque sea igual,
     * lo vuelve a encender.
     */
    static final class PosServo {
        double position;
        Double cached = null;
        boolean pwmOn = true;
        boolean everCommanded = false;
        PwmControl.PwmRange range = PwmControl.PwmRange.defaultRange;

        Servo proxy() {
            return (Servo) Proxy.newProxyInstance(Fakes.class.getClassLoader(),
                    new Class[]{Servo.class, PwmControl.class},
                    (p, m, a) -> {
                        switch (m.getName()) {
                            case "setPosition": {
                                double v = (Double) a[0];
                                position = v;
                                everCommanded = true;
                                if (cached == null || cached != v) {
                                    cached = v;
                                    pwmOn = true;
                                }
                                return null;
                            }
                            case "getPosition": return position;
                            case "setPwmRange": range = (PwmControl.PwmRange) a[0]; return null;
                            case "getPwmRange": return range;
                            case "setPwmEnable": pwmOn = true; return null;
                            case "setPwmDisable": if (pwmOn) cached = null; pwmOn = false; return null;
                            case "isPwmEnabled": return pwmOn;
                            case "getDirection": return Servo.Direction.FORWARD;
                            case "toString": return "FakeServo";
                            case "hashCode": return System.identityHashCode(p);
                            case "equals": return p == a[0];
                            default: return defaultFor(m.getReturnType());
                        }
                    });
        }

        /** Pulso que está mandando el hub, en µs (NaN si está apagado). */
        double us() {
            if (!pwmOn || !everCommanded) return Double.NaN;
            return range.usPulseLower + position * (range.usPulseUpper - range.usPulseLower);
        }
    }

    // ---------------- servo continuo ----------------
    /** Mismas reglas que PosServo: setPower es un comando de posición del servo. */
    static final class ContServo {
        double power;
        Double cached = null;
        boolean pwmOn = true;
        PwmControl.PwmRange range = PwmControl.PwmRange.defaultRange;

        CRServo proxy() {
            return (CRServo) Proxy.newProxyInstance(Fakes.class.getClassLoader(),
                    new Class[]{CRServo.class, PwmControl.class},
                    (p, m, a) -> {
                        switch (m.getName()) {
                            case "setPower": {
                                double v = (Double) a[0];
                                power = v;
                                if (cached == null || cached != v) {
                                    cached = v;
                                    pwmOn = true;
                                }
                                return null;
                            }
                            case "getPower": return power;
                            case "setPwmRange": range = (PwmControl.PwmRange) a[0]; return null;
                            case "getPwmRange": return range;
                            case "setPwmEnable": pwmOn = true; return null;
                            case "setPwmDisable": if (pwmOn) cached = null; pwmOn = false; return null;
                            case "isPwmEnabled": return pwmOn;
                            case "toString": return "FakeCRServo";
                            case "hashCode": return System.identityHashCode(p);
                            case "equals": return p == a[0];
                            default: return defaultFor(m.getReturnType());
                        }
                    });
        }

        /** Potencia efectiva: con el PWM apagado no se mueve. */
        double effective() {
            return pwmOn ? power : 0;
        }
    }

    // ---------------- entrada digital ----------------
    static DigitalChannel digital(BooleanSupplier state) {
        return (DigitalChannel) Proxy.newProxyInstance(Fakes.class.getClassLoader(), new Class[]{DigitalChannel.class},
                (p, m, a) -> {
                    switch (m.getName()) {
                        case "getState": return state.getAsBoolean();
                        case "toString": return "FakeDigital";
                        case "hashCode": return System.identityHashCode(p);
                        case "equals": return p == a[0];
                        default: return defaultFor(m.getReturnType());
                    }
                });
    }
}
