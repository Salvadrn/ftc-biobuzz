package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.sensors.RevColorReader;
import org.firstinspires.ftc.teamcode.util.ColorClassifier;
import org.firstinspires.ftc.teamcode.vision.RivalNectarDetector;

/**
 * Muestra todos los sensores en vivo. No mueve nada.
 * Lo que no esté en la configuración sale como "no configurado" en vez de tronar.
 *
 * Sirve para calibrar SensorCfg:
 *  - Pon una POLLEN, una NECTAR roja y una azul frente a cada V3 y anota hue, saturación
 *    y distancia. Ajusta UPPER, COPA y THROAT y los rangos de ColorClassifier. En la
 *    garganta, prueba cada pelota pegada a cada pared de la fila (CALC: NECTAR de 8.3 a
 *    14.1 mm y POLLEN de 6.3 a 33.1): ahí el tamaño no separa una de otra.
 *  - Corta cada break-beam con la mano: tiene que decir "cortado". Si dice lo contrario,
 *    cambia BEAM_ACTIVE_HIGH. Si no lee estable (el BB1 mide 258 mm de LED a LED), una
 *    resistencia de 10 kΩ de la señal a 3.3 V.
 *  - Con el tubo vacío, cada V3 tiene que dar color distinto de 0. "SIN LECTURA" es cable
 *    o I2C. "SATURADO" = bajar SensorCfg.COLOR_GAIN.
 *  - La C270 usa los rangos genéricos del SDK: revisa que vea una NECTAR azul a ~60 cm y
 *    que no vea las cintas del piso (para la vista en vivo, IntakeCameraCfg.LIVE_VIEW).
 */
@TeleOp(name = "Test: Sensores", group = "2 Calibracion")
public class TestSensors extends LinearOpMode {
    @Override
    public void runOpMode() {
        BallSensor upper = color(Names.COLOR_UPPER, SensorCfg.UPPER);
        BallSensor copa = color(Names.COLOR_COPA, SensorCfg.COPA);
        BallSensor throat = color(Names.COLOR_THROAT, SensorCfg.THROAT);
        BeamSensor mouth = beam(Names.BEAM_MOUTH, SensorCfg.BEAM_ACTIVE_HIGH);
        BeamSensor throatBeam = beam(Names.BEAM_THROAT, SensorCfg.BEAM_ACTIVE_HIGH);
        BeamSensor filaBeam = beam(Names.BEAM_FILA, SensorCfg.BEAM_ACTIVE_HIGH);
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, Names.LIMELIGHT);
        if (limelight != null) limelight.start();
        RivalNectarDetector camera = new RivalNectarDetector(hardwareMap, Alliance.RED);

        waitForStart();
        LLStatus st = null;
        long lastStatusNs = 0;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            line("V3 arriba (z 128)", upper, now);
            line("V3 copa (z 52)", copa, now);
            line("V3 garganta", throat, now);
            telemetry.addData("BB1 boca", state(mouth, "cortado", "libre"));
            telemetry.addData("BB garganta", state(throatBeam, "cortado", "libre"));
            telemetry.addData("BB3 fila", state(filaBeam, "cortado", "libre"));

            if (limelight == null) {
                telemetry.addData("Limelight", "no configurado");
            } else {
                if (st == null || now - lastStatusNs > 1_000_000_000L) {   // GET HTTP que bloquea: 1 por segundo
                    st = limelight.getStatus();
                    lastStatusNs = now;
                }
                LLResult r = limelight.getLatestResult();
                telemetry.addData("Limelight", "%s  pipeline %d  %.0f fps  %.0f °C",
                        limelight.isConnected() ? "conectado" : "DESCONECTADO",
                        st.getPipelineIndex(), st.getFps(), st.getTemp());
                if (r != null && r.isValid()) {
                    telemetry.addData("  tags", "%d   ts %.0f", r.getFiducialResults().size(), r.getTimestamp());
                }
            }
            telemetry.addData("C270 (busca NECTAR azul)", camera.isPresent()
                    ? (camera.rivalAhead() ? "VE UNA  " : "nada  ") + camera.describe() : "no configurada");

            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                telemetry.addData(hub.getDeviceName(), "%.2f V   %.2f A",
                        hub.getInputVoltage(VoltageUnit.VOLTS), hub.getCurrent(CurrentUnit.AMPS));
            }
            telemetry.update();
        }
        if (limelight != null) limelight.stop();
        // La C270 no se cierra aquí: el SDK cierra el VisionPortal al terminar el OpMode.
    }

    private BallSensor color(String name, ColorClassifier.Thresholds t) {
        if (!exists(name)) return null;
        return new BallSensor(new RevColorReader(hardwareMap, name, SensorCfg.COLOR_GAIN), t, 0.0);
    }

    private BeamSensor beam(String name, boolean activeHigh) {
        DigitalChannel ch = hardwareMap.tryGet(DigitalChannel.class, name);
        return ch == null ? null : new BeamSensor(ch, activeHigh);
    }

    private boolean exists(String name) {
        return hardwareMap.tryGet(HardwareDevice.class, name) != null;
    }

    private void line(String label, BallSensor s, long now) {
        if (s == null) {
            telemetry.addData(label, "no configurado");
            return;
        }
        s.read(now);
        telemetry.addData(label, s.describe());
    }

    private static String state(BeamSensor b, String on, String off) {
        if (b == null) return "no configurado";
        return b.isActive() ? on : off;
    }
}
