package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.intake.TongueServo;

import java.util.List;

/**
 * Calibra el intake de la lengua. Solo necesita "lengua" (servo 2000-0025-0002) e "intake"
 * (M_F). Al dar Play no se mueve nada: el servo arranca suelto. Sin pelotas en el intake.
 *
 * ARMADO (CAD): el estriado del servo se cala con la manivela a medio camino entre abajo y
 * plegada mientras el servo recibe 1500 µs. El sentido del PWM no está verificado: si la
 * lengua va al revés, los µs se invierten alrededor de 1500.
 *
 * PROCEDIMIENTO (los µs cambian en vivo; al final se copian a IntakeLenguaCfg)
 *  1. X: plegar con la secuencia del robot. Ajusta FOLDED_US con dpad arriba/abajo (±10 µs)
 *     o bumpers (±1 µs) hasta que quede recargada en el tope de atrás SIN empujarlo (la
 *     corriente del hub no sube). Y la suelta: tiene que quedarse ahí sola.
 *  2. B (solo desde plegada): al empuje. Ajusta PUSH_US hasta el punto en que la biela
 *     apenas toca la lengua, sin moverla.
 *  3. A: desplegar con la secuencia del robot (rápido al empuje y en rampa hasta abajo). La
 *     lengua tiene que bajar sin golpe. Ajusta DOWN_US: la cola en el tope de abajo sin
 *     empujarlo.
 *  4. Gatillo derecho: M_F a IN_POWER. Las bandas de abajo tienen que jalar hacia adentro;
 *     si sacan, IntakeLenguaCfg.REVERSED = true. Gatillo izquierdo: reversa.
 *  5. Mide cuánto tarda cada movimiento y ajusta SERVO_DEG_S (más bajo = más pesimista).
 *
 * El ajuste solo mueve el valor de donde está la lengua ahorita: así nunca da un salto grande
 * (un salto de plegada a abajo la dejaría caer).
 */
@TeleOp(name = "Tune: Lengua", group = "2 Calibracion")
public class TuneLengua extends LinearOpMode {
    private enum Field { FOLDED, PUSH, DOWN }

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, Names.TONGUE_SERVO);
        TongueServo tongue = new TongueServo(servo);
        DcMotor motor = hardwareMap.get(DcMotor.class, Names.INTAKE_MOTOR);
        motor.setDirection(IntakeLenguaCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addLine("Al dar Play no se mueve nada. X pliega, A despliega (con la secuencia).");
        telemetry.update();
        waitForStart();

        Field field = null;          // qué valor se ajusta: el de donde está la lengua
        String note = "";
        double lastMotor = Double.NaN;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            if (gamepad1.xWasPressed()) {
                tongue.fold(now);
                field = Field.FOLDED;
            }
            if (gamepad1.aWasPressed()) {
                tongue.deploy(now);
                field = Field.DOWN;
            }
            if (gamepad1.bWasPressed()) {
                if (field == Field.FOLDED || field == Field.PUSH) {    // de plegada al empuje la lengua no se mueve
                    tongue.manual(IntakeLenguaCfg.PUSH_US, now);
                    field = Field.PUSH;
                    note = "";
                } else {
                    note = "B solo desde plegada (X primero)";
                }
            }
            if (gamepad1.yWasPressed()) tongue.release();

            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step = 10;
            if (gamepad1.dpadDownWasPressed()) step = -10;
            if (gamepad1.rightBumperWasPressed()) step = 1;
            if (gamepad1.leftBumperWasPressed()) step = -1;
            if (step != 0 && field != null) {
                if (!tongue.isSettled()) {
                    note = "espera a que llegue antes de ajustar";
                } else {
                    switch (field) {
                        case FOLDED: IntakeLenguaCfg.FOLDED_US = Range.clip(IntakeLenguaCfg.FOLDED_US + step, 500, 2500); break;
                        case PUSH: IntakeLenguaCfg.PUSH_US = Range.clip(IntakeLenguaCfg.PUSH_US + step, 500, 2500); break;
                        case DOWN: IntakeLenguaCfg.DOWN_US = Range.clip(IntakeLenguaCfg.DOWN_US + step, 500, 2500); break;
                    }
                    tongue.manual(us(field), now);
                    note = "";
                }
            }
            tongue.update(now);

            double m = 0;
            if (gamepad1.right_trigger > 0.3) m = IntakeLenguaCfg.IN_POWER;
            if (gamepad1.left_trigger > 0.3) m = -IntakeLenguaCfg.IN_POWER;
            if (m != lastMotor) {
                motor.setPower(m);
                lastMotor = m;
            }

            telemetry.addData("Lengua", tongue.describe());
            telemetry.addData("Ajustando", field == null ? "nada" : field + String.format("  %.0f µs  (%.1f° desde 1500 µs)",
                    us(field), (us(field) - 1500) * TongueServo.DEG_PER_US));
            telemetry.addData("FOLDED / PUSH / DOWN", "%.0f / %.0f / %.0f µs",
                    IntakeLenguaCfg.FOLDED_US, IntakeLenguaCfg.PUSH_US, IntakeLenguaCfg.DOWN_US);
            telemetry.addData("M_F", m > 0 ? "jalando" : m < 0 ? "reversa" : "parado (BRAKE)");
            for (LynxModule hub : hubs) {
                telemetry.addData("Corriente " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
            }
            if (!note.isEmpty()) telemetry.addLine("!! " + note);
            telemetry.addLine();
            telemetry.addLine("X plegar  A desplegar  B empuje  Y soltar   dpad ±10 µs  bumpers ±1 µs");
            telemetry.addLine("gatillo der. jalar   gatillo izq. reversa");
            telemetry.update();
        }
        motor.setPower(0);
    }

    private static double us(Field f) {
        switch (f) {
            case PUSH: return IntakeLenguaCfg.PUSH_US;
            case DOWN: return IntakeLenguaCfg.DOWN_US;
            case FOLDED:
            default: return IntakeLenguaCfg.FOLDED_US;
        }
    }
}
