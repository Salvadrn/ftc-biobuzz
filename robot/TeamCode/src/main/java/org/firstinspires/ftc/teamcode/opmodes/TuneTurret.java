package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Mueve el servo de la torreta paso a paso para sacar los números de Config.TurretCfg.
 * Solo necesita el servo "turret" en la configuración.
 *
 * PROCEDIMIENTO
 *  1. Con el dpad llevar la torreta a que mire exacto al frente. Anotar la posición:
 *     ese número es SERVO_CENTER.
 *  2. Girarla 90 grados a la IZQUIERDA, medido con escuadra. Anotar la posición.
 *     DEG_PER_SERVO_UNIT = 90 / |posición izquierda - SERVO_CENTER|
 *     Si para ir a la izquierda la posición BAJÓ, poner SERVO_INVERTED = true.
 *  3. Llevarla a cada tope seguro, antes de jalar cables. Los ángulos que marca la
 *     telemetría son MIN_ANGLE_DEG y MAX_ANGLE_DEG. Primero hay que poner los números
 *     de los pasos 1 y 2 en Config y volver a cargar, o el ángulo mostrado no sirve.
 *
 * CONTROLES:  dpad izq/der = paso de 0.01    bumpers = paso de 0.001    Y = ir a SERVO_CENTER
 */
@TeleOp(name = "Tune: Torreta", group = "2 Calibracion")
public class TuneTurret extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, Names.TURRET_SERVO);
        double position = TurretCfg.SERVO_CENTER;

        telemetry.addLine("Al dar Play el servo va a SERVO_CENTER (" + position + ").");
        telemetry.addLine("Revisar que nada estorbe el giro.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadLeftWasPressed()) position -= 0.01;
            if (gamepad1.dpadRightWasPressed()) position += 0.01;
            if (gamepad1.leftBumperWasPressed()) position -= 0.001;
            if (gamepad1.rightBumperWasPressed()) position += 0.001;
            if (gamepad1.yWasPressed()) position = TurretCfg.SERVO_CENTER;

            position = Range.clip(position, 0, 1);
            servo.setPosition(position);

            telemetry.addData("Posición del servo", "%.3f", position);
            telemetry.addData("Ángulo según Config", "%.1f grados  (+ = izquierda)", Turret.servoToAngle(position));
            telemetry.addLine();
            telemetry.addLine("dpad = 0.01   bumpers = 0.001   Y = centro");
            telemetry.update();
        }
    }
}
