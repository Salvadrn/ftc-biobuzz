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
 * SERVO_CENTER (unos ±35°). Para soltar todo el recorrido hay que sostener los dos
 * bumpers y apretar X. Hazlo con el USB del Limelight desconectado o la cámara sin
 * montar, para que el cable no se jale si algo está mal.
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
            if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.xWasPressed()) unlocked = !unlocked;

            double lo = unlocked ? 0 : TurretCfg.SERVO_CENTER - 0.15;
            double hi = unlocked ? 1 : TurretCfg.SERVO_CENTER + 0.15;
            position = Range.clip(position, lo, hi);
            servo.setPosition(position);

            double us = TurretCfg.PWM_MIN_US + position * (TurretCfg.PWM_MAX_US - TurretCfg.PWM_MIN_US);
            double angle = Turret.servoToAngle(position);
            telemetry.addData("Posición del servo", "%.3f   (%.0f µs)", position, us);
            telemetry.addData("Ángulo según Config", "%.1f°  (+ = izquierda)", angle);
            telemetry.addData("Recorrido", unlocked ? "SUELTO: cuidado con los cables" : "bloqueado a ±0.15");
            if (Math.abs(angle) > TurretCfg.MAX_ANGLE_DEG) telemetry.addLine("!! Más allá de ±105°: se jalan los cables");
            telemetry.addLine();
            telemetry.addLine("dpad = 0.01   bumpers = 0.001   Y = centro   A/B = ±90°");
            telemetry.addLine("sostener los 2 bumpers + X = soltar/bloquear");
            telemetry.update();
        }
    }
}
