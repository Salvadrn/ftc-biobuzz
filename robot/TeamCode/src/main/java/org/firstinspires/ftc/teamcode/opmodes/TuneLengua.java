package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.intake.TongueServo;

import java.util.List;

/**
 * Calibra el intake de la lengua con el procedimiento del documento del CAD v138 (4.3).
 * Solo necesita "lengua" (servo 2000-0025-0002) e "intake" (M_F). Al dar Play no se mueve
 * nada: el servo arranca suelto. Sin pelotas en el intake. Los pasos 0 a 4 son sin M_F.
 *
 * Los µs de IntakeLenguaCfg son de cálculo (1002 / 1516 / 1998) y el calado del estriado
 * puede quedar hasta ±48 µs fuera (medio diente de 25): más que los 36 µs que hay entre
 * plegada y el punto en que el servo jala la lengua contra su tope cerca del punto muerto de
 * la biela (~410-520 N). Por eso aquí nunca se manda de golpe un valor de cálculo cerca de un
 * tope. "Hacia abajo" es el sentido de FOLDED_US a DOWN_US: dpad y bumpers mueven siempre en
 * ese sentido (arriba / derecho) o en el contrario (abajo / izquierdo).
 *
 *  0. CALADO. dpad izquierdo manda SPLINE_SET_US (1500) fijo. Con la lengua plegada a mano
 *     contra su tope de atrás, mete la manivela en el diente (o diente + cara del REX: 150
 *     posiciones cada 2.4°) más cercano a "hacia atrás, 48° abajo de la horizontal". Si la
 *     lengua no asienta en su tope, la manivela quedó más de 2.35° hacia abajo: usa otra.
 *  1. SENTIDO. Desde el calado, dpad arriba (5 µs hacia abajo) un par de veces: la manivela
 *     tiene que BAJAR (girar hacia el frente). Si sube, el sentido está al revés: el botón del
 *     stick izquierdo refleja los tres valores alrededor de SPLINE_SET_US y regresa al calado.
 *     No uses "la lengua no se movió" como criterio: con el calado corrido, el empuje llega
 *     hasta 63 µs del calado aunque el sentido esté bien.
 *  2. EMPUJE. Sigue hacia abajo (bumper derecho, 1 µs) hasta que la biela apenas toque la
 *     lengua sin moverla. B lo marca: PUSH_US = ese valor, y recorre los otros dos con las
 *     diferencias de cálculo, que no dependen del calado (FOLDED_US = PUSH_US - 514 y
 *     DOWN_US = PUSH_US + 482, en el sentido de abajo). PUSH_US - 1516 es el error de calado.
 *  3. ABAJO, por despegue. A despliega con la secuencia del robot (rápido al empuje y en
 *     rampa); la primera vez se detiene 100 µs antes de DOWN_US, con la lengua un poco arriba
 *     de su tope. Baja de 1 en 1 µs hasta que la cola toque el tope de abajo: ahí despega. B
 *     lo marca: DOWN_US = ese + 46 µs hacia abajo. La corriente no sirve aquí: por la ranura
 *     el servo nunca empuja ese tope.
 *  4. PLEGADA, desde la vertical (con la lengua abajo). X la sube; la primera vez se detiene
 *     100 µs antes de FOLDED_US, con la lengua antes de la vertical y la biela tensa. Acércate
 *     a plegada (dpad abajo 5 µs, luego bumper izquierdo 1 µs) hasta que la lengua pase la
 *     vertical y caiga sola a su tope de atrás. B lo marca: FOLDED_US = ese + 47 µs hacia
 *     plegada. Comprueba a mano: la punta avanza ~4 mm y se traba. Si avanza 8 mm o más,
 *     faltan µs hacia plegada; si no avanza nada y el servo zumba, se pasó: regresa al menos
 *     20 µs. NUNCA guardes un valor con el que la corriente del hub sube con la lengua en su
 *     tope (la corriente sube 36 µs después del punto correcto, ya cerca del punto muerto).
 *  5. M_F, con la lengua abajo: gatillo derecho jala a IN_POWER; el tramo de abajo de las
 *     bandas tiene que jalar hacia adentro (+Y). Si saca, IntakeLenguaCfg.REVERSED = true.
 *     Gatillo izquierdo: reversa. Detén una pelota con la mano: la corriente de M_F y su
 *     límite (MF_CURRENT_LIMIT_A) se ven abajo.
 *  6. VELOCIDAD: ya marcadas abajo y plegada, A y X usan la secuencia completa. Cronometra
 *     el plegado y ajusta SERVO_DEG_S (más bajo = más pesimista).
 *  7. Copia los µs a IntakeLenguaCfg y pon CALIBRATED = true. Sin eso el TeleOp no mueve la
 *     lengua ni M_F.
 *
 * Ya marcado un valor, los bumpers y el dpad lo ajustan en vivo donde está la lengua.
 * Criterios (CALC, no dependen del calado): PUSH_US - FOLDED_US ≈ 514, DOWN_US - PUSH_US ≈ 482
 * y el error de calado dentro de ±48 µs (±8 con el REX).
 */
@TeleOp(name = "Tune: Lengua", group = "2 Calibracion")
public class TuneLengua extends LinearOpMode {
    /** Lo que marca B y lo que ajustan dpad y bumpers: dónde está la lengua. */
    private enum Field { PUSH, DOWN, FOLDED }

    // CALC del CAD (documento v138, 4.3): no dependen del calado del estriado
    private static final double FOLD_TO_PUSH_US = 514;        // PUSH_US - FOLDED_US
    private static final double PUSH_TO_DOWN_US = 482;        // DOWN_US - PUSH_US
    private static final double PUSH_FROM_SET_US = 16;        // empuje desde el calado ideal
    private static final double LIFTOFF_TO_DOWN_US = 46;      // del despegue a DOWN_US
    private static final double VERTICAL_TO_FOLDED_US = 47;   // de la vertical a FOLDED_US (47-48)
    private static final double FIRST_MARGIN_US = 100;        // la primera vez, A y X se quedan antes
    private static final double PUSH_SEARCH_US = 70;          // el empuje cae a -33..+63 µs del calado
    private static final double DOWN_BEYOND_US = 15;          // la junta llega a 17 µs más allá de abajo
    private static final double FOLDED_BEYOND_US = 20;        // a 36 µs más allá de plegada jala contra el tope

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, Names.TONGUE_SERVO);
        TongueServo tongue = new TongueServo(servo);
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Names.INTAKE_MOTOR);
        motor.setDirection(IntakeLenguaCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setCurrentAlert(IntakeLenguaCfg.MF_CURRENT_LIMIT_A, CurrentUnit.AMPS);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addLine("Al dar Play no se mueve nada. Paso 0: dpad izquierdo = calado (SPLINE_SET_US).");
        telemetry.update();
        waitForStart();

        Field field = null;              // dónde está la lengua: lo que marca B
        double us = Double.NaN;          // lo último que se mandó
        boolean pushMarked = false, downMarked = false, foldedMarked = false;
        String note = "";
        double lastMotor = Double.NaN;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            // Sentido de abajo en µs: +1 con la ficha vista desde el estriado, -1 reflejado
            double down = IntakeLenguaCfg.DOWN_US >= IntakeLenguaCfg.FOLDED_US ? 1 : -1;

            // 0. Calado
            if (gamepad1.dpadLeftWasPressed()) {
                if (field == Field.DOWN) {
                    note = "el calado es con la lengua plegada: X primero";
                } else {
                    us = IntakeLenguaCfg.SPLINE_SET_US;
                    tongue.manual(us, now);
                    field = Field.PUSH;
                    note = "";
                }
            }

            // 1. Sentido al revés: reflejar los tres valores alrededor del calado
            if (gamepad1.leftStickButtonWasPressed()) {
                if (field == Field.PUSH && !downMarked && !foldedMarked) {
                    double c = IntakeLenguaCfg.SPLINE_SET_US;
                    IntakeLenguaCfg.FOLDED_US = 2 * c - IntakeLenguaCfg.FOLDED_US;
                    IntakeLenguaCfg.PUSH_US = 2 * c - IntakeLenguaCfg.PUSH_US;
                    IntakeLenguaCfg.DOWN_US = 2 * c - IntakeLenguaCfg.DOWN_US;
                    pushMarked = false;
                    us = c;
                    tongue.manual(us, now);
                    note = "valores reflejados y de vuelta en el calado: prueba otra vez el sentido";
                } else {
                    note = "reflejar solo en el calado, antes de marcar abajo y plegada";
                }
            }

            // 3. Desplegar con la secuencia (la primera vez, 100 µs antes de DOWN_US)
            if (gamepad1.aWasPressed()) {
                if (!pushMarked) {
                    note = "marca primero el empuje (paso 2): el salto rápido va hasta PUSH_US";
                } else if (field == Field.DOWN) {
                    note = "ya está abajo";
                } else {
                    if (downMarked) {
                        us = IntakeLenguaCfg.DOWN_US;
                        tongue.deploy(now);
                    } else {
                        us = IntakeLenguaCfg.DOWN_US - down * FIRST_MARGIN_US;
                        tongue.deployForTuning(us, now);
                    }
                    field = Field.DOWN;
                    note = "";
                }
            }

            // 4. Plegar (la primera vez, desde abajo y 100 µs antes de FOLDED_US)
            if (gamepad1.xWasPressed()) {
                if (foldedMarked) {
                    us = IntakeLenguaCfg.FOLDED_US;
                    tongue.fold(now);
                    field = Field.FOLDED;
                    note = "";
                } else if (field != Field.DOWN) {
                    note = "la primera vez se pliega desde abajo (A, paso 3)";
                } else {
                    us = IntakeLenguaCfg.FOLDED_US + down * FIRST_MARGIN_US;
                    tongue.manual(us, now);
                    field = Field.FOLDED;
                    note = "";
                }
            }
            if (gamepad1.yWasPressed()) tongue.release();

            // Pasos chicos desde donde está: + = hacia abajo
            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step = 5;
            if (gamepad1.dpadDownWasPressed()) step = -5;
            if (gamepad1.rightBumperWasPressed()) step = 1;
            if (gamepad1.leftBumperWasPressed()) step = -1;
            if (step != 0 && field != null) {
                double next = Range.clip(us + down * step, TongueServo.MIN_US, TongueServo.MAX_US);
                // Con el empuje marcado, los topes de cálculo ya no dependen del calado
                double maxDown = IntakeLenguaCfg.PUSH_US + down * (PUSH_TO_DOWN_US + DOWN_BEYOND_US);
                double maxFold = IntakeLenguaCfg.PUSH_US - down * (FOLD_TO_PUSH_US + FOLDED_BEYOND_US);
                if (!tongue.isSettled()) {
                    note = "espera a que llegue antes de ajustar";
                } else if (field == Field.PUSH && Math.abs(next - IntakeLenguaCfg.SPLINE_SET_US) > PUSH_SEARCH_US) {
                    note = "a más de 70 µs del calado la lengua llegaría a la vertical y caería: revisa el calado";
                } else if (pushMarked && down * (next - maxDown) > 0) {
                    note = "más allá de abajo no hay tope en el CAD (la junta da 17 µs más)";
                } else if (pushMarked && down * (maxFold - next) > 0) {
                    note = "más allá de plegada - 20 µs: cerca del punto muerto de la biela (jala contra el tope a -36)";
                } else {
                    us = next;
                    tongue.manual(us, now);
                    if (field == Field.PUSH && pushMarked) IntakeLenguaCfg.PUSH_US = us;
                    if (field == Field.DOWN && downMarked) IntakeLenguaCfg.DOWN_US = us;
                    if (field == Field.FOLDED && foldedMarked) IntakeLenguaCfg.FOLDED_US = us;
                    note = "";
                }
            }

            // Marcar
            if (gamepad1.bWasPressed()) {
                if (field == null) {
                    note = "primero el calado (dpad izquierdo)";
                } else if (!tongue.isSettled()) {
                    note = "espera a que llegue";
                } else if (field == Field.PUSH) {
                    IntakeLenguaCfg.PUSH_US = us;
                    if (!foldedMarked) IntakeLenguaCfg.FOLDED_US = us - down * FOLD_TO_PUSH_US;
                    if (!downMarked) IntakeLenguaCfg.DOWN_US = us + down * PUSH_TO_DOWN_US;
                    pushMarked = true;
                    note = "empuje marcado; plegada y abajo recorridas con las diferencias de cálculo";
                } else if (field == Field.DOWN) {
                    if (downMarked) {
                        note = "abajo ya está marcado: ajústalo con los bumpers";
                    } else {
                        IntakeLenguaCfg.DOWN_US = us + down * LIFTOFF_TO_DOWN_US;
                        us = IntakeLenguaCfg.DOWN_US;
                        tongue.manual(us, now);
                        downMarked = true;
                        note = "abajo = despegue + 46 µs";
                    }
                } else {
                    if (foldedMarked) {
                        note = "plegada ya está marcada: ajústala con los bumpers";
                    } else {
                        IntakeLenguaCfg.FOLDED_US = us - down * VERTICAL_TO_FOLDED_US;
                        us = IntakeLenguaCfg.FOLDED_US;
                        tongue.manual(us, now);
                        foldedMarked = true;
                        note = "plegada = vertical + 47 µs. A mano: la punta avanza ~4 mm y se traba";
                    }
                }
            }
            tongue.update(now);

            // 5. M_F, solo con la lengua abajo
            double m = 0;
            if (gamepad1.right_trigger > 0.3) m = IntakeLenguaCfg.IN_POWER;
            if (gamepad1.left_trigger > 0.3) m = -IntakeLenguaCfg.IN_POWER;
            if (m != 0 && field != Field.DOWN) {
                m = 0;
                note = "M_F solo con la lengua abajo (paso 3)";
            }
            if (m != lastMotor) {
                motor.setPower(m);
                lastMotor = m;
            }

            double nominalPush = IntakeLenguaCfg.SPLINE_SET_US + down * PUSH_FROM_SET_US;
            double servoUsS = IntakeLenguaCfg.SERVO_DEG_S / TongueServo.DEG_PER_US;
            double foldS = Math.abs(IntakeLenguaCfg.DOWN_US - IntakeLenguaCfg.FOLDED_US) / servoUsS;
            double deployS = Math.abs(IntakeLenguaCfg.PUSH_US - IntakeLenguaCfg.FOLDED_US) / servoUsS
                    + Math.abs(IntakeLenguaCfg.DOWN_US - IntakeLenguaCfg.PUSH_US)
                    / (IntakeLenguaCfg.DEPLOY_RAMP_DEG_S / TongueServo.DEG_PER_US);

            telemetry.addData("Lengua", tongue.describe());
            telemetry.addData("Dónde (B marca)", field == null ? "nada: dpad izquierdo = calado"
                    : String.format("%s  %.0f µs", field, us));
            telemetry.addData("FOLDED / PUSH / DOWN", "%.0f%s / %.0f%s / %.0f%s µs",
                    IntakeLenguaCfg.FOLDED_US, foldedMarked ? " (marcado)" : "",
                    IntakeLenguaCfg.PUSH_US, pushMarked ? " (marcado)" : "",
                    IntakeLenguaCfg.DOWN_US, downMarked ? " (marcado)" : "");
            telemetry.addData("Revisión (CALC)", "PUSH-FOLDED %.0f (≈514)  DOWN-PUSH %.0f (≈482)  calado %+.0f µs (≤ ±48)",
                    down * (IntakeLenguaCfg.PUSH_US - IntakeLenguaCfg.FOLDED_US),
                    down * (IntakeLenguaCfg.DOWN_US - IntakeLenguaCfg.PUSH_US),
                    down * (IntakeLenguaCfg.PUSH_US - nominalPush));
            telemetry.addData("Sentido", down > 0 ? "más µs = hacia abajo (ficha vista desde el estriado)"
                    : "menos µs = hacia abajo (reflejado)");
            telemetry.addData("Modelo (SERVO_DEG_S %.0f)", "plegar %.2f s, desplegar %.2f s",
                    IntakeLenguaCfg.SERVO_DEG_S, foldS, deployS);
            telemetry.addData("M_F", "%s  %.2f A%s", m > 0 ? "jalando" : m < 0 ? "reversa" : "parado (BRAKE)",
                    motor.getCurrent(CurrentUnit.AMPS), motor.isOverCurrent()
                            ? String.format("  ARRIBA DE %.1f A", IntakeLenguaCfg.MF_CURRENT_LIMIT_A) : "");
            for (LynxModule hub : hubs) {
                telemetry.addData("Corriente " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
            }
            if (!note.isEmpty()) telemetry.addLine("!! " + note);
            telemetry.addLine();
            telemetry.addLine("dpad izq calado   stick izq reflejar   A desplegar   X plegar   B marcar   Y soltar");
            telemetry.addLine("dpad arriba/abajo 5 µs (arriba = hacia abajo)   bumpers 1 µs (der = hacia abajo)");
            telemetry.addLine("gatillo der jalar   gatillo izq reversa (con la lengua abajo)");
            telemetry.addLine("Al final: copia los µs a IntakeLenguaCfg y pon CALIBRATED = true");
            telemetry.update();
        }
        motor.setPower(0);
    }
}
