package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Calibra el servo de 5 vueltas de la torreta. Solo necesita "turret" en la configuración.
 *
 * SEGURIDAD: con los valores de Config todavía sin medir, "±105°" puede no ser ±105° de
 * verdad. Por eso este OpMode arranca BLOQUEADO a ±0.15 de posición alrededor de
 * SERVO_CENTER (unos ±35°). Sosteniendo los dos bumpers y apretando X se suelta hasta el
 * límite de Config (MIN/MAX_ANGLE_DEG, ±105°), nunca hasta el final del servo (±115.7°,
 * más que la guía de cables y la junta, ±110°). Hazlo con el USB del Limelight
 * desconectado o la cámara sin montar, para que el cable no se jale si algo está mal.
 *
 * ARMADO: antes de engranar el piñón con la corona, corre este OpMode (al dar Play el
 * servo va a SERVO_CENTER, 1500 µs de fábrica) y engrana con la torreta mirando al frente.
 * Si se engrana en otro punto, el centro físico queda corrido y "±105°" ya no es seguro.
 *
 * PROCEDIMIENTO
 *  1. dpad izq/der (0.01) y bumpers (0.001) hasta que la torreta mire exacto al frente.
 *     Esa posición es SERVO_CENTER.
 *  2. Sube la posición un poco: si la torreta gira a la DERECHA, SERVO_INVERTED = true.
 *  3. Pon los valores en Config y vuelve a cargar. Con A la torreta va a +90° y con B a
 *     -90° según Config: mide con escuadra. Si no da 90, DEG_PER_SERVO_UNIT está mal
 *     (de cálculo es 231.4).
 *  4. Con el recorrido suelto, revisa a mano que en ±105° la guía de cables no jale nada.
 */
@TeleOp(name = "Tune: Torreta", group = "2 Calibracion")
public class TuneTurret extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, Names.TURRET_SERVO);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(TurretCfg.PWM_MIN_US, TurretCfg.PWM_MAX_US));
        }
        double position = TurretCfg.SERVO_CENTER;
        boolean unlocked = false;

        telemetry.addLine("Al dar Play el servo va a SERVO_CENTER (" + position + ").");
        telemetry.addLine("Limelight desconectado o sin montar mientras calibras.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadLeftWasPressed()) position -= 0.01;
            if (gamepad1.dpadRightWasPressed()) position += 0.01;
            if (gamepad1.leftBumperWasPressed()) position -= 0.001;
            if (gamepad1.rightBumperWasPressed()) position += 0.001;
            if (gamepad1.yWasPressed()) position = TurretCfg.SERVO_CENTER;
            if (gamepad1.aWasPressed()) position = Turret.angleToServo(90);
            if (gamepad1.bWasPressed()) position = Turret.angleToServo(-90);
            // X se lee en cada ciclo: el SDK guarda un X viejo hasta que alguien lo lee
            boolean x = gamepad1.xWasPressed();
            if (x && gamepad1.left_bumper && gamepad1.right_bumper) unlocked = !unlocked;

            double lo, hi;
            if (unlocked) {   // hasta el límite de Config, nunca hasta el final del servo
                double a = Turret.angleToServo(TurretCfg.MIN_ANGLE_DEG);
                double b = Turret.angleToServo(TurretCfg.MAX_ANGLE_DEG);
                lo = Math.min(a, b);
                hi = Math.max(a, b);
            } else {
                lo = TurretCfg.SERVO_CENTER - 0.15;
                hi = TurretCfg.SERVO_CENTER + 0.15;
            }
            position = Range.clip(position, lo, hi);
            servo.setPosition(position);

            double us = TurretCfg.PWM_MIN_US + position * (TurretCfg.PWM_MAX_US - TurretCfg.PWM_MIN_US);
            double angle = Turret.servoToAngle(position);
            telemetry.addData("Posición del servo", "%.3f   (%.0f µs)", position, us);
            telemetry.addData("Ángulo según Config", "%.1f°  (+ = izquierda)", angle);
            telemetry.addData("Recorrido", unlocked ? "SUELTO hasta ±105° de Config: cuidado con los cables"
                    : "bloqueado a ±0.15");
            telemetry.addLine();
            telemetry.addLine("dpad = 0.01   bumpers = 0.001   Y = centro   A/B = ±90°");
            telemetry.addLine("sostener los 2 bumpers + X = soltar/bloquear");
            telemetry.update();
        }
    }
}
