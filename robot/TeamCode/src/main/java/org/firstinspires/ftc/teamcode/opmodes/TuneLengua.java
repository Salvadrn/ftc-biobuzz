package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.Config.Names;

import java.util.List;

/**
 * Calibra el intake de la lengua. Solo necesita "lengua" (servo 2000-0025-0002) e "intake"
 * (M_F). Al dar Play no se mueve nada: el servo arranca suelto.
 *
 * PROCEDIMIENTO (los µs cambian en vivo; al final se copian a IntakeLenguaCfg)
 *  1. Con la lengua plegada a mano, X manda FOLDED_US. Ajusta con dpad arriba/abajo (±10 µs)
 *     y bumpers (±1 µs) hasta que quede recargada en el tope de atrás SIN empujarlo (la
 *     corriente del hub no sube). Y la suelta: tiene que quedarse ahí sola.
 *  2. A manda DOWN_US: lengua en el tope duro de abajo, sin empujarlo. Mismo ajuste.
 *  3. Gatillo derecho: M_F a IN_POWER. Las bandas de abajo tienen que jalar hacia adentro;
 *     si sacan, IntakeLenguaCfg.REVERSED = true. Gatillo izquierdo: reversa.
 *  4. Mide cuánto tarda la lengua de plegada a abajo: eso (con margen) es TRAVEL_S.
 *
 * Nunca plegarla con una pelota en la garganta: los dedos la empujarían contra la flecha
 * del pivote de la fila. El sentido del PWM no está verificado: los primeros pasos, cortos.
 */
@TeleOp(name = "Tune: Lengua", group = "2 Calibracion")
public class TuneLengua extends LinearOpMode {
    private enum Field { FOLDED, DOWN }

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, Names.TONGUE_SERVO);
        PwmControl pwm = servo instanceof PwmControl ? (PwmControl) servo : null;
        if (pwm != null) pwm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        DcMotor motor = hardwareMap.get(DcMotor.class, Names.INTAKE_MOTOR);
        motor.setDirection(IntakeLenguaCfg.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addLine("Al dar Play no se mueve nada. X / A mandan plegada / abajo.");
        telemetry.update();
        waitForStart();

        Field field = null;          // qué posición se está ajustando; null = servo suelto
        double lastMotor = Double.NaN;
        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) field = Field.FOLDED;
            if (gamepad1.aWasPressed()) field = Field.DOWN;
            if (gamepad1.yWasPressed()) {
                field = null;
                if (pwm != null) pwm.setPwmDisable();
            }

            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step = 10;
            if (gamepad1.dpadDownWasPressed()) step = -10;
            if (gamepad1.rightBumperWasPressed()) step = 1;
            if (gamepad1.leftBumperWasPressed()) step = -1;
            if (field != null) {
                switch (field) {
                    case FOLDED: IntakeLenguaCfg.FOLDED_US = Range.clip(IntakeLenguaCfg.FOLDED_US + step, 500, 2500); break;
                    case DOWN: IntakeLenguaCfg.DOWN_US = Range.clip(IntakeLenguaCfg.DOWN_US + step, 500, 2500); break;
                }
                servo.setPosition((us(field) - 500) / 2000.0);   // un cambio vuelve a prender el PWM
            }

            double m = 0;
            if (gamepad1.right_trigger > 0.3) m = IntakeLenguaCfg.IN_POWER;
            if (gamepad1.left_trigger > 0.3) m = -IntakeLenguaCfg.IN_POWER;
            if (m != lastMotor) {
                motor.setPower(m);
                lastMotor = m;
            }

            telemetry.addData("Ajustando", field == null ? "nada (servo suelto)" : field + String.format("  %.0f µs", us(field)));
            telemetry.addData("FOLDED_US / DOWN_US", "%.0f / %.0f", IntakeLenguaCfg.FOLDED_US, IntakeLenguaCfg.DOWN_US);
            telemetry.addData("Ángulo del servo desde 1500 µs", field == null ? "-" : String.format("%.1f°", (us(field) - 1500) * 0.15));
            telemetry.addData("M_F", m > 0 ? "jalando" : m < 0 ? "reversa" : "parado (BRAKE)");
            for (LynxModule hub : hubs) {
                telemetry.addData("Corriente " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
            }
            telemetry.addLine();
            telemetry.addLine("X plegada  A abajo  Y soltar   dpad ±10 µs  bumpers ±1 µs");
            telemetry.addLine("gatillo der. jalar   gatillo izq. reversa");
            telemetry.update();
        }
        motor.setPower(0);
    }

    private static double us(Field f) {
        return f == Field.DOWN ? IntakeLenguaCfg.DOWN_US : IntakeLenguaCfg.FOLDED_US;
    }
}
