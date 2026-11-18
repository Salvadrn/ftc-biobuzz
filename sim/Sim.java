import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.Config.IntakeF2Cfg;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.Config.VisionCfg;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeF2;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.ColorClassifier;
import org.firstinspires.ftc.teamcode.util.InterpTable;
import org.firstinspires.ftc.teamcode.util.StateStore;
import org.firstinspires.ftc.teamcode.vision.Cell;
import org.firstinspires.ftc.teamcode.vision.CellTarget;
import org.firstinspires.ftc.teamcode.vision.CellVision;
import org.firstinspires.ftc.teamcode.vision.ClusterSolver;
import org.firstinspires.ftc.teamcode.vision.HiveGeometry;
import org.firstinspires.ftc.teamcode.vision.TagSighting;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Pruebas en la computadora, sin robot. Todo con tiempo virtual (no depende de qué tan
 * cargada esté la máquina) y con las clases REALES del robot y del SDK:
 *  - el Limelight sale de proyectar el campo 3D del manual TU09 y pasa por LLResult del SDK;
 *  - torreta, hood, alimentador e intake corren contra hardware falso con física simple.
 * Nada de esto sustituye probar en el robot: los números de Config siguen sin medir.
 */
public class Sim {
    static int fails = 0;
    static final long MS = 1_000_000L;

    static void check(String name, boolean ok, String detail) {
        System.out.printf("%s %-62s %s%n", ok ? "ok   " : "FALLA", name, detail);
        if (!ok) fails++;
    }

    public static void main(String[] args) {
        System.out.println("== color");
        testColor();
        System.out.println("== tabla");
        testInterp();
        System.out.println("== geometría de cámara");
        testRayRoundTrip();
        System.out.println("== centro de la fila con tags tapados (proyección 3D)");
        testClusterSolver3d();
        System.out.println("== CELL de arriba y distancia, de punta a punta con LLResult del SDK");
        testUpDown(false);
        testUpDown(true);
        System.out.println("== tiempos del Limelight");
        testVisionTiming();
        System.out.println("== torreta en lazo cerrado (Limelight simulado, retraso, chasis girando)");
        for (double speed : new double[]{140, 95, 190}) testTurretLoop(speed);
        testTurretWrap(360);
        testTurretWrap(450);
        testTurretInitSettle();
        System.out.println("== hood: ley de los dos servos");
        testHood();
        System.out.println("== alimentador: fila, copa y elevador");
        testFeeder();
        System.out.println("== intake F2");
        testIntakeParking();
        testIntakeRejectAndUnjam();

        System.out.println(fails == 0 ? "\nTODO OK" : "\nFALLAS: " + fails);
        System.exit(fails == 0 ? 0 : 1);
    }

    // =========================================================== color
    static void testColor() {
        ColorClassifier.Thresholds t = SensorCfg.COPA;
        check("amarillo = POLLEN", ColorClassifier.classify(0.6, 0.55, 0.1, 12, t) == BallType.POLLEN, "");
        check("rojo = NECTAR roja", ColorClassifier.classify(0.6, 0.1, 0.1, 5, t) == BallType.NECTAR_RED, "");
        check("azul = NECTAR azul", ColorClassifier.classify(0.1, 0.25, 0.7, 5, t) == BallType.NECTAR_BLUE, "");
        check("gris cerca = desconocida grande (gap de NECTAR)",
                ColorClassifier.classify(0.3, 0.3, 0.3, 5, t) == BallType.UNKNOWN, "");
        check("gris lejos = POLLEN por tamaño", ColorClassifier.classify(0.3, 0.3, 0.3, 16, t) == BallType.POLLEN, "");
        check("nada enfrente", ColorClassifier.classify(0.6, 0.1, 0.1, 80, t) == BallType.NONE, "");
        check("desconocida usa gap de NECTAR", BallType.UNKNOWN.needsNectarGap(), "");
        check("NECTAR azul es rival para rojo", BallType.NECTAR_BLUE.isRivalFor(Alliance.RED), "");
    }

    // =========================================================== tabla
    static void testInterp() {
        InterpTable t = new InterpTable(new double[][]{{30, 2400}, {50, 2700}, {70, 3000}});
        check("interpola en medio", Math.abs(t.get(40) - 2550) < 1e-9, "");
        check("no extrapola abajo", Math.abs(t.get(10) - 2400) < 1e-9, "");
        check("no extrapola arriba", Math.abs(t.get(99) - 3000) < 1e-9, "");
    }

    // =========================================================== cámara
    static void testRayRoundTrip() {
        double worst = 0;
        for (double f : new double[]{20, 60, 120})
            for (double l : new double[]{-30, 0, 25})
                for (double u : new double[]{5, 20, 40}) {
                    double[] txty = HiveGeometry.project(f, l, u, 30);
                    if (txty == null) continue;
                    HiveGeometry.Ray r = HiveGeometry.ray(txty[0], txty[1], 30);
                    worst = Math.max(worst, Math.abs(r.azimuthLeftDeg - Math.toDegrees(Math.atan2(l, f))));
                    worst = Math.max(worst, Math.abs(r.elevationDeg - Math.toDegrees(Math.atan2(u, Math.hypot(f, l)))));
                }
        check("tx,ty -> rumbo y elevación con pitch 30°", worst < 1e-9, String.format("peor %.2e°", worst));
        HiveGeometry.Ray r = HiveGeometry.ray(10, 10, 30);
        check("con pitch, tx 10° pide más de 10° de giro", r.azimuthLeftDeg < -12.5,
                String.format("rumbo %.2f° para tx 10°, ty 10°", r.azimuthLeftDeg));
    }

    // =========================================================== solver
    static void testClusterSolver3d() {
        boolean[][] masks = {{true, true, true, true}, {true, true, false, false}, {false, false, true, true},
                {true, false, false, true}, {false, true, true, false}};
        Random rnd = new Random(3);
        double worstIn = 0;
        String worstCase = "";
        for (double dist : new double[]{30, 60, 100})
            for (double phi : new double[]{0, 30, -45, 60}) {
                double[] rc = FieldSim.rowCenter(true);
                double ax = rc[0] + dist * Math.cos(Math.toRadians(phi));
                double ay = rc[1] + dist * Math.sin(Math.toRadians(phi));
                double heading = Math.atan2(rc[1] - ay, rc[0] - ax) + Math.toRadians(8);
                FieldSim.Cam cam = new FieldSim.Cam(ax, ay, heading, VisionCfg.CAMERA_BEHIND_AXIS_IN,
                        VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
                double[] truthCam = cam.toCam(rc);
                double truthTx = Math.toDegrees(Math.atan(truthCam[0] / truthCam[2]));
                for (boolean[] m : masks) {
                    List<TagSighting> tags = new ArrayList<>();
                    for (int i = 0; i < 4; i++) {
                        if (!m[i]) continue;
                        FieldSim.Seen s = FieldSim.project(cam, new double[]{rc[0], rc[1] + Cell.TAG_X_IN[i], rc[2]},
                                30 + i, rnd, 0.0, 0.0);
                        if (s != null) tags.add(new TagSighting(i, s.tx, s.ty, s.ta, Double.NaN));
                    }
                    if (tags.size() < 2) continue;
                    ClusterSolver.Solution sol = new ClusterSolver(1.0).solve(Cell.RED_SCORING, tags, 0);
                    double errIn = Math.abs(Math.tan(Math.toRadians(sol.txDeg - truthTx))) * Math.hypot(truthCam[0], truthCam[2]);
                    if (errIn > worstIn) {
                        worstIn = errIn;
                        worstCase = String.format("dist %.0f, φ %.0f°", dist, phi);
                    }
                }
            }
        check("centro de la fila sin ruido, peor caso", worstIn < 0.8, String.format("%.2f in (%s)", worstIn, worstCase));
    }

    // =========================================================== arriba/abajo
    static void testUpDown(boolean full3d) {
        int saved = VisionCfg.UPDOWN_CONFIRM_FRAMES;
        VisionCfg.UPDOWN_CONFIRM_FRAMES = 1;
        Random rnd = new Random(11);
        int right = 0, wrong = 0, undecided = 0, total = 0, uncentered = 0;
        double worstDist = 0, worstBear = 0, worstLat = 0;
        String wrongCases = "";
        for (double dist = 26; dist <= 130; dist += 4)
            for (double phi : new double[]{0, 20, -35, 50, -60})
                for (double aimOff : new double[]{0, 12, -18}) {
                    double[] rc = FieldSim.rowCenter(true);
                    double ax = rc[0] + dist * Math.cos(Math.toRadians(phi));
                    double ay = rc[1] + dist * Math.sin(Math.toRadians(phi));
                    double heading = Math.atan2(rc[1] - ay, rc[0] - ax) + Math.toRadians(aimOff);
                    FieldSim.Cam cam = new FieldSim.Cam(ax, ay, heading, VisionCfg.CAMERA_BEHIND_AXIS_IN,
                            VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
                    LLResult r = FieldSim.result(cam, Cell.RED_SCORING, 1000, 5, full3d, rnd, 0.1, 0.15);
                    if (r == null || r.getFiducialResults().isEmpty()) continue;
                    total++;
                    CellVision v = new CellVision(Alliance.RED);
                    CellTarget t = v.process(r, 1_000_000_000L);
                    Cell up = v.upCell();
                    if (up == null) { undecided++; continue; }
                    if (up != Cell.RED_SCORING) {
                        wrong++;
                        if (wrongCases.length() < 160) wrongCases += String.format(" [d%.0f φ%.0f off%.0f]", dist, phi, aimOff);
                        continue;
                    }
                    right++;
                    if (t != null && !t.centered) uncentered++;
                    if (t != null && t.centered) {
                        double[] truth = FieldSim.truthFromAxis(ax, ay, heading);
                        worstBear = Math.max(worstBear, Math.abs(t.bearingDeg - truth[0]));
                        worstLat = Math.max(worstLat, truth[1] * Math.abs(Math.tan(Math.toRadians(t.bearingDeg - truth[0]))));
                        if (dist >= 30 && dist <= 110) worstDist = Math.max(worstDist, Math.abs(t.distanceIn - truth[1]));
                    }
                }
        String tag = full3d ? "con Full 3D" : "sin 3D";
        check("CELL de arriba nunca confundida con la de abajo (" + tag + ")", wrong == 0,
                String.format("bien %d, mal %d, sin decidir %d de %d%s", right, wrong, undecided, total, wrongCases));
        check("puntería al centro de la fila (" + tag + ")", worstLat < 1.5,
                String.format("peor %.2f in de lado (%.2f°); %d fotos de un solo tag sin centrar", worstLat, worstBear, uncentered));
        check("distancia de 30 a 110 in (" + tag + ")", worstDist < 3.0, String.format("peor %.2f in", worstDist));
        VisionCfg.UPDOWN_CONFIRM_FRAMES = saved;
    }

    // =========================================================== tiempos
    static void testVisionTiming() {
        Random rnd = new Random(5);
        double[] rc = FieldSim.rowCenter(true);
        FieldSim.Cam cam = new FieldSim.Cam(rc[0] + 60, rc[1], Math.PI, VisionCfg.CAMERA_BEHIND_AXIS_IN,
                VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
        CellVision v = new CellVision(Alliance.RED);
        long now = 5_000_000_000L;
        v.flipHive();   // que no tenga que decidir: el piloto dice cuál está arriba
        if (v.upCell() != Cell.RED_SCORING) v.flipHive();

        LLResult a = FieldSim.result(cam, Cell.RED_SCORING, 1234, 100, false, rnd, 0, 0);
        CellTarget t1 = v.process(a, now);
        check("foto nueva se usa", t1 != null, "");
        if (t1 != null) {
            double ageMs = (now - t1.captureTimeNs) / 1e6;
            check("instante de la foto = ahora - (staleness + cl + tl)", Math.abs(ageMs - 135) < 5,
                    String.format("%.1f ms (esperado 135)", ageMs));
        }
        LLResult same = FieldSim.result(cam, Cell.RED_SCORING, 1234, 0, false, rnd, 0, 0);
        check("la misma foto (mismo ts) no se usa dos veces", v.process(same, now + 10 * MS) == null, "");
        LLResult old = FieldSim.result(cam, Cell.RED_SCORING, 1300, 300, false, rnd, 0, 0);
        check("foto con más de 250 ms se descarta", v.process(old, now + 20 * MS) == null, "");
        LLResult fresh = FieldSim.result(cam, Cell.RED_SCORING, 1333, 3, false, rnd, 0, 0);
        check("la siguiente foto sí se usa", v.process(fresh, now + 30 * MS) != null, "");
    }

    // =========================================================== torreta
    static final class TurretWorld {
        final Fakes.PosServo servo = new Fakes.PosServo();
        final Turret turret = new Turret(servo.proxy());
        final CellVision vision = new CellVision(Alliance.RED);
        final Random rnd = new Random(7);
        double trueTurret, yaw = 180, realSpeed;
        double axisX, axisY;
        long now = 0;
        double nextFrameNs = 0;
        final ArrayList<double[]> pending = new ArrayList<>();   // {disponible, capturaTs, yawCap, torretaCap}
        final ArrayList<double[]> history = new ArrayList<>();   // {t, yaw, torreta}

        TurretWorld(double realSpeed, double startTurret) {
            this.realSpeed = realSpeed;
            this.trueTurret = startTurret;
            double[] rc = FieldSim.rowCenter(true);
            axisX = rc[0] + 60;
            axisY = rc[1] + 20;
            vision.flipHive();
            if (vision.upCell() != Cell.RED_SCORING) vision.flipHive();
        }

        double errorDeg() {
            double heading = Math.toRadians(yaw + trueTurret);
            return FieldSim.truthFromAxis(axisX, axisY, heading)[0];
        }

        /** Avanza 10 ms con el chasis girando a yawRate °/s. */
        void step(double yawRate, boolean autoAim, double manual) {
            double dt = 0.010;
            now += 10 * MS;
            yaw += yawRate * dt;
            double cmd = servo.everCommanded ? Turret.servoToAngle(servo.position) : trueTurret;
            double s = realSpeed * dt;
            trueTurret += Math.max(-s, Math.min(s, cmd - trueTurret));
            history.add(new double[]{now, yaw, trueTurret});

            // El Limelight toma una foto cada 33 ms y la publica 35 ms después (+ hasta 10 de consulta)
            if (now >= nextFrameNs) {
                nextFrameNs = now + 33 * MS;
                pending.add(new double[]{now + 35 * MS + rnd.nextInt(10) * MS, now / 1e6, yaw, trueTurret});
            }
            CellTarget frame = null;
            for (int i = 0; i < pending.size(); i++) {
                double[] p = pending.get(i);
                if (p[0] <= now) {
                    pending.remove(i--);
                    FieldSim.Cam cam = new FieldSim.Cam(axisX, axisY, Math.toRadians(p[2] + p[3]),
                            VisionCfg.CAMERA_BEHIND_AXIS_IN, VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
                    long staleMs = (long) ((now - p[0]) / 1e6);
                    LLResult r = FieldSim.result(cam, Cell.RED_SCORING, p[1], staleMs, false, rnd, 0.1, 0.1);
                    CellTarget t = vision.process(r, now);
                    if (t != null) frame = t;
                }
            }
            turret.update(now, yaw, frame, autoAim, manual);
        }
    }

    static void testTurretLoop(double realSpeed) {
        TurretWorld w = new TurretWorld(realSpeed, 0);
        w.turret.startCentering(w.now);
        w.servo.everCommanded = true;
        double worstRest = 0, worstSpin = 0;
        int reversals = 0;
        double prevCmd = 0, prevDir = 0;
        boolean lockedEarly = false;
        for (int i = 0; i < 450; i++) {
            double t = i * 0.01;
            double rate = (t > 2.5 && t < 3.5) ? 90 : 0;
            w.step(rate, true, 0);
            double err = Math.abs(w.errorDeg());
            if ((t > 2.0 && t < 2.5) || t > 4.2) worstRest = Math.max(worstRest, err);
            if (t > 2.5 && t < 3.5) worstSpin = Math.max(worstSpin, err);
            if (w.turret.isLocked(w.now) && err > TurretCfg.LOCK_TOLERANCE_DEG + 1.0) lockedEarly = true;
            double cmd = w.turret.getCommandedDeg();
            double dir = Math.signum(cmd - prevCmd);
            if (dir != 0) {
                if (prevDir != 0 && dir != prevDir && (t < 2.5 || t > 3.7)) reversals++;
                prevDir = dir;
            }
            prevCmd = cmd;
        }
        String name = String.format("servo real a %.0f°/s (modelo %.0f)", realSpeed, TurretCfg.MAX_SPEED_DEG_S);
        check(name + ": error en reposo", worstRest < 1.0, String.format("peor %.2f°", worstRest));
        check(name + ": error con el chasis girando 90°/s", worstSpin < 6.0, String.format("peor %.2f°", worstSpin));
        check(name + ": no oscila", reversals <= 6, reversals + " cambios de sentido");
        check(name + ": nunca dice 'fijo' con error grande", !lockedEarly, "");
        check(name + ": fijo al final", w.turret.isLocked(w.now), "");
    }

    static void testTurretWrap(double degrees) {
        TurretWorld w = new TurretWorld(140, 0);
        w.turret.startCentering(w.now);
        w.servo.everCommanded = true;
        for (int i = 0; i < 200; i++) w.step(0, true, 0);
        int spinSteps = (int) Math.round(degrees / 120.0 / 0.01);
        for (int i = 0; i < spinSteps; i++) w.step(120, true, 0);
        for (int i = 0; i < 300; i++) w.step(0, true, 0);
        double err = Math.abs(w.errorDeg());
        boolean reachable = Math.abs(Turret.wrap180(FieldSim.truthFromAxis(w.axisX, w.axisY, Math.toRadians(w.yaw))[0]))
                <= TurretCfg.MAX_ANGLE_DEG;
        check(String.format("chasis da %.0f° y la torreta vuelve al objetivo", degrees),
                !reachable || (err < 1.5 && w.turret.hasTarget()),
                String.format("error final %.2f°, torreta en %.1f°", err, w.trueTurret));
    }

    static void testTurretInitSettle() {
        TurretWorld w = new TurretWorld(140, 80);   // quedó girada 80° del partido anterior
        w.turret.startCentering(w.now);
        boolean lockedWhileUnknown = false;
        for (int i = 0; i < 300; i++) {
            w.step(0, true, 0);
            if (!w.turret.isPositionKnown() && w.turret.isLocked(w.now)) lockedWhileUnknown = true;
        }
        check("arranque: no confía en la posición mientras centra", !lockedWhileUnknown, "");
        check("arranque: después de centrar, apunta bien", Math.abs(w.errorDeg()) < 1.0 && w.turret.isLocked(w.now),
                String.format("error %.2f°", w.errorDeg()));
    }

    // =========================================================== hood
    static double gapDegFromUs(double us) {
        return (us - HoodCfg.GAP_POLLEN_US) / (HoodCfg.GAP_NECTAR_US - HoodCfg.GAP_POLLEN_US) * HoodCfg.GAP_NECTAR_DEG;
    }

    static double angleDegFromUs(double us) {
        return (us - HoodCfg.ANGLE_ZERO_US) / HoodCfg.ANGLE_US_PER_DEG;
    }

    /** Límites de choque del CAD v104 (δ efectivo) según el gap, interpolados. */
    static double limelightLimit(double gap) {
        return interp3(gap, -30.8, -24.9, -18.3);
    }

    static double flatTopLimit(double gap) {
        return interp3(gap, 35.4, 40.1, 44.5);
    }

    static double interp3(double gap, double atPollen, double atMid, double atNectar) {
        double t = Math.max(0, Math.min(1, gap / HoodCfg.GAP_NECTAR_DEG));   // 0 POLLEN, 1 NECTAR
        return t <= 0.5 ? atPollen + (atMid - atPollen) * t / 0.5 : atMid + (atNectar - atMid) * (t - 0.5) / 0.5;
    }

    /**
     * Hood con dos servos de verdad: cada uno sigue su comando con retraso (constante de
     * tiempo tau) y velocidad máxima. Regresa {margen mínimo a la Limelight, margen mínimo a
     * las FlatTop, gap del primer comando, ángulo del primer comando, posiciones ok, listo,
     * tiempo máximo de un cambio de gap}. gapAlive/angleAlive = false simula un servo muerto.
     */
    static double[] runHood(double gapSpeed, double gapTau, double angleSpeed, double angleTau,
                            boolean gapAlive, boolean angleAlive, double gapRate) {
        double savedRate = HoodCfg.GAP_RATE_DEG_S;
        HoodCfg.GAP_RATE_DEG_S = gapRate;
        Fakes.PosServo gs = new Fakes.PosServo(), as = new Fakes.PosServo();
        StateStore store = new StateStore.Memory();
        store.putDouble("hood.gap", HoodCfg.GAP_NECTAR_DEG);   // quedó en NECTAR
        store.putDouble("hood.delta", 0);
        Hood hood = new Hood(gs.proxy(), as.proxy(), store);
        double gap = HoodCfg.GAP_NECTAR_DEG, ang = -HoodCfg.GAP_NECTAR_DEG;   // donde está de verdad
        long now = 0;
        hood.start(now);
        double firstGap = gapDegFromUs(gs.us()), firstAng = angleDegFromUs(as.us());
        double minLl = 99, minFt = 99, worstChange = 0;
        boolean posOk = true;
        Object[][] plan = {{BallType.POLLEN, 0.0}, {BallType.NECTAR_RED, 0.0}, {BallType.NECTAR_RED, -10.0},
                {BallType.POLLEN, -10.0}, {BallType.NECTAR_BLUE, -5.0}, {BallType.POLLEN, 0.0},
                {BallType.NECTAR_RED, -10.0}, {BallType.POLLEN, 0.0}};
        for (Object[] step : plan) {
            hood.setGapFor((BallType) step[0]);
            hood.setDelta((Double) step[1]);
            double changeTime = 0;
            boolean ready = false;
            for (int i = 0; i < 300; i++) {
                now += 5 * MS;
                hood.update(now);
                double dt = 0.005;
                double gTarget = gapDegFromUs(gs.us()), aTarget = angleDegFromUs(as.us());
                if (gapAlive) gap = follow(gap, gTarget, gapSpeed, gapTau, dt);
                if (angleAlive) ang = follow(ang, aTarget, angleSpeed, angleTau, dt);
                double delta = gap + ang;
                minLl = Math.min(minLl, delta - limelightLimit(gap));
                minFt = Math.min(minFt, flatTopLimit(gap) - delta);
                posOk &= gs.position >= 0 && gs.position <= 1 && as.position >= 0 && as.position <= 1;
                if (!ready && hood.isReady(now)) { ready = true; changeTime = i * dt; }
            }
            worstChange = Math.max(worstChange, ready ? changeTime : 99);
        }
        HoodCfg.GAP_RATE_DEG_S = savedRate;
        return new double[]{minLl, minFt, firstGap, firstAng, posOk ? 1 : 0, hood.isReady(now) ? 1 : 0, worstChange};
    }

    /** Servo: primer orden con constante tau y velocidad máxima. */
    static double follow(double x, double target, double maxSpeed, double tau, double dt) {
        double want = (target - x) * (1 - Math.exp(-dt / tau));
        double lim = maxSpeed * dt;
        return x + Math.max(-lim, Math.min(lim, want));
    }

    static void testHood() {
        double[] r = runHood(180, 0.03, 240, 0.03, true, true, HoodCfg.GAP_RATE_DEG_S);
        check("arranca mandando donde quedó (NECTAR), sin brinco",
                Math.abs(r[2] - HoodCfg.GAP_NECTAR_DEG) < 0.01 && Math.abs(r[3] + HoodCfg.GAP_NECTAR_DEG) < 0.01,
                String.format("gap %.1f°, ángulo %.1f°", r[2], r[3]));
        check("servos parejos: lejos de la Limelight y de las FlatTop", r[0] > 8 && r[1] > 20,
                String.format("margen %.1f° a la Limelight, %.1f° a las FlatTop", r[0], r[1]));
        check("posiciones de servo siempre dentro de 0..1", r[4] == 1, "");
        check("listo al terminar", r[5] == 1, "");
        check("un cambio de gap tarda menos de 1.2 s", r[6] < 1.2, String.format("%.2f s el peor", r[6]));
        double[] gapLate = runHood(120, 0.10, 240, 0.02, true, true, HoodCfg.GAP_RATE_DEG_S);
        check("servo de gap lento y atrasado: sigue lejos de los dos", gapLate[0] > 5 && gapLate[1] > 15,
                String.format("margen %.1f° / %.1f°", gapLate[0], gapLate[1]));
        double[] angLate = runHood(240, 0.02, 120, 0.10, true, true, HoodCfg.GAP_RATE_DEG_S);
        check("servo de ángulo lento y atrasado: sigue lejos de los dos", angLate[0] > 5 && angLate[1] > 15,
                String.format("margen %.1f° / %.1f°", angLate[0], angLate[1]));
        // Un servo muerto a media transición: sin sensor de posición no se puede evitar.
        // Se reporta para el CAD: un tope físico de δ protegería la Limelight.
        double[] gapDead = runHood(180, 0.03, 240, 0.03, false, true, HoodCfg.GAP_RATE_DEG_S);
        double[] angDead = runHood(180, 0.03, 240, 0.03, true, false, HoodCfg.GAP_RATE_DEG_S);
        System.out.printf("      (referencia, servo de gap muerto: margen %.1f° a la Limelight y %.1f° a las FlatTop)%n",
                gapDead[0], gapDead[1]);
        System.out.printf("      (referencia, servo de ángulo muerto: margen %.1f° a la Limelight y %.1f° a las FlatTop)%n",
                angDead[0], angDead[1]);
    }

    // =========================================================== alimentador
    static final class BallWorld {
        BallType throat = BallType.NONE, fila = BallType.NONE, copa = BallType.NONE, upper = BallType.NONE;
        double copaMm = 0;
        double filaPushS = 0, liftS = 0, fireS = 0;
        int launched = 0;
        boolean hazard = false;
        String hazardWhy = "";
        boolean dip = false;
    }

    static void testFeeder() {
        BallWorld bw = new BallWorld();
        bw.throat = BallType.POLLEN;
        bw.fila = BallType.NECTAR_RED;
        bw.copa = BallType.POLLEN;
        bw.upper = BallType.NECTAR_RED;

        Fakes.ContServo fila = new Fakes.ContServo();
        Fakes.PosServo copa = new Fakes.PosServo();
        Fakes.Motor elev = new Fakes.Motor();
        BallSensor upperS = new BallSensor(out -> {
            fill(out, bw.upper, 20);
            return true;
        }, SensorCfg.UPPER, 0.04);
        BallSensor copaS = new BallSensor(out -> {
            fill(out, bw.copaMm < 10 ? bw.copa : BallType.NONE, bw.copa.isNectar() ? 4.6 : 16.1);
            return true;
        }, SensorCfg.COPA, 0.04);
        BeamSensor filaB = new BeamSensor(Fakes.digital(() -> bw.fila.isBall()), true);
        Feeder f = new Feeder(fila.proxy(), copa.proxy(), elev.proxy(), upperS, copaS, filaB);
        long now = 0;
        f.initHardware(now);
        ArrayList<BallType> order = new ArrayList<>();
        double t = 0;
        int shotsCounted = 0;
        boolean gapWrong = false;
        BallType firingTruth = BallType.NONE;
        Feeder.State prevState = f.getState();
        while (t < 15 && bw.launched < 4) {
            now += 10 * MS;
            t += 0.01;
            if (f.hasUpperBall() && !f.isBusy()) f.requestShot();
            BallType upperBefore = bw.upper;
            f.update(now, true, false, bw.dip);
            bw.dip = false;
            shotsCounted += f.consumeShots();
            if (f.getState() == Feeder.State.FIRING && prevState != Feeder.State.FIRING) firingTruth = upperBefore;
            // Mientras dispara, el gap tiene que ser el de la pelota que sale, aunque el V3 ya no la vea
            if (f.getState() == Feeder.State.FIRING && f.nextBall() != firingTruth) gapWrong = true;
            prevState = f.getState();
            physics(bw, fila, copa, elev, 0.01, order);
        }
        for (int i = 0; i < 80; i++) {          // que se asiente
            now += 10 * MS;
            f.update(now, false, false, bw.dip);
            bw.dip = false;
            shotsCounted += f.consumeShots();
            physics(bw, fila, copa, elev, 0.01, order);
        }
        check("salen las 4 pelotas", bw.launched == 4, bw.launched + " en " + String.format("%.1f s", t));
        check("el orden es arriba, copa, fila, garganta", order.toString().equals(
                "[NECTAR_RED, POLLEN, NECTAR_RED, POLLEN]"), order.toString());
        check("cuenta 4 tiros (G407)", shotsCounted == 4, shotsCounted + " tiros");
        check("nunca sube la copa con pelota esperando arriba", !bw.hazard, bw.hazardWhy);
        check("mientras dispara, el gap no cambia aunque el V3 ya no vea la pelota", !gapWrong, "");
        check("al terminar la fila queda apagada (PWM off)", !fila.pwmOn, "");
        check("al terminar la copa queda abajo y suelta", bw.copaMm < 0.5 && !copa.pwmOn,
                String.format("%.1f mm, PWM %s", bw.copaMm, copa.pwmOn ? "on" : "off"));
    }

    static void fill(double[] out, BallType b, double mm) {
        switch (b) {
            case POLLEN: out[0] = 0.6; out[1] = 0.55; out[2] = 0.1; break;
            case NECTAR_RED: out[0] = 0.6; out[1] = 0.1; out[2] = 0.1; break;
            case NECTAR_BLUE: out[0] = 0.1; out[1] = 0.25; out[2] = 0.7; break;
            default: out[0] = 0.05; out[1] = 0.05; out[2] = 0.05; mm = 200;
        }
        out[3] = mm;
    }

    static void physics(BallWorld bw, Fakes.ContServo fila, Fakes.PosServo copa, Fakes.Motor elev,
                        double dt, List<BallType> order) {
        // Copa: el servo sigue el pulso a ~236 mm/s; sin PWM se queda (abajo, en el piso)
        if (copa.pwmOn && copa.everCommanded) {
            double us = copa.us();
            double targetMm = (FeederCfg.COPA_DOWN_US - us) / (FeederCfg.COPA_DOWN_US - FeederCfg.COPA_UP_US) * FeederCfg.COPA_TRAVEL_MM;
            double s = 236 * dt;
            bw.copaMm += Math.max(-s, Math.min(s, targetMm - bw.copaMm));
        }
        if (bw.copaMm > 57) { bw.hazard = true; bw.hazardWhy = "copa contra el tope"; }
        double ePow = elev.power;
        if (bw.copaMm > 40 && bw.upper.isBall() && bw.copa.isBall() && Math.abs(ePow) < 0.05) {
            bw.hazard = true;
            bw.hazardWhy = String.format("copa a %.0f mm con pelota arriba y motor parado", bw.copaMm);
        }

        // Fila
        double fp = fila.effective() * Math.signum(FeederCfg.FILA_POWER_IN);
        if (fp > 0.5) {
            bw.filaPushS += dt;
            if (bw.filaPushS >= 0.3) {
                if (bw.fila.isBall() && !bw.copa.isBall() && bw.copaMm < 1) {
                    bw.copa = bw.fila; bw.fila = BallType.NONE; bw.filaPushS = 0;
                } else if (!bw.fila.isBall() && bw.throat.isBall()) {
                    bw.fila = bw.throat; bw.throat = BallType.NONE; bw.filaPushS = 0;
                }
            }
        } else {
            bw.filaPushS = 0;
        }

        // Elevador
        if (ePow > 0.5 && bw.upper.isBall()) {
            bw.fireS += dt;
            if (bw.fireS >= 0.12) {
                order.add(bw.upper);
                bw.upper = BallType.NONE;
                bw.launched++;
                bw.dip = true;
                bw.fireS = 0;
            }
        } else {
            bw.fireS = 0;
        }
        if (ePow > 0.1 && bw.copaMm >= 50 && bw.copa.isBall() && !bw.upper.isBall()) {
            bw.liftS += dt;
            if (bw.liftS >= 0.25) {
                bw.upper = bw.copa; bw.copa = BallType.NONE; bw.liftS = 0;
            }
        } else {
            bw.liftS = 0;
        }
    }

    // =========================================================== intake
    static final class Rotor {
        final Fakes.Motor motor = new Fakes.Motor();
        final double startDeg, magnetDeg;
        boolean magnetState = false, magnetReported = false;

        Rotor(double startDeg, double magnetDeg, boolean staleFirstReading) {
            this.startDeg = startDeg;
            this.magnetDeg = magnetDeg;
            boolean inside = Math.abs(wrap(startDeg - magnetDeg)) < 9;
            magnetState = magnetReported = inside && !staleFirstReading;
            if (staleFirstReading) magnetState = inside;
        }

        double angleDeg() {
            return startDeg + motor.pos / IntakeF2Cfg.TICKS_PER_ROTOR_REV * 360.0;
        }

        static double wrap(double d) {
            d = d % 360;
            if (d > 180) d -= 360;
            if (d <= -180) d += 360;
            return d;
        }

        void step(double dt) {
            motor.step(dt, 1150.0 / 60.0 * 145.1);
            double off = Math.abs(wrap(angleDeg() - magnetDeg));
            magnetReported = magnetState;          // un ciclo de retraso del sensor
            if (!magnetState && off < 9) magnetState = true;
            else if (magnetState && off > 11) magnetState = false;   // histéresis
        }
    }

    static void testIntakeParking() {
        double worst = 0, worstFull = 0;
        boolean allParked = true;
        for (double start : new double[]{0, 100, 170, -120, 3})
            for (double mag : new double[]{0, 137, -60})
            for (boolean stale : new boolean[]{false, true}) {
                Rotor rot = new Rotor(start, mag, stale);
                BeamSensor magnet = new BeamSensor(Fakes.digital(() -> !rot.magnetReported), false);
                BeamSensor none1 = new BeamSensor(Fakes.digital(() -> false), true);
                BeamSensor none2 = new BeamSensor(Fakes.digital(() -> false), true);
                BallSensor noColor = new BallSensor(out -> { fill(out, BallType.NONE, 0); return true; }, SensorCfg.THROAT, 0.04);
                IntakeF2 in = new IntakeF2(rot.motor.proxy(), magnet, none1, none2, noColor, null, Alliance.RED);
                long now = 0;
                boolean done = false;
                for (int i = 0; i < 1000 && !done; i++) {
                    now += 15 * MS;
                    done = in.initStep(now);
                    rot.step(0.015);
                }
                for (int i = 0; i < 20; i++) rot.step(0.015);
                double err = Rotor.wrap(rot.angleDeg() - mag - IntakeF2Cfg.INDEX_OFFSET_DEG);
                allParked &= in.isParked();
                worst = Math.max(worst, Math.abs(err));

                in.setCommand(Intake.Command.STOP);
                for (int i = 0; i < 150; i++) {
                    now += 15 * MS;
                    in.update(now, true);
                    rot.step(0.015);
                }
                double fullErr = Rotor.wrap(rot.angleDeg() - mag - IntakeF2Cfg.INDEX_OFFSET_DEG - IntakeF2Cfg.FULL_HOLD_DEG);
                worstFull = Math.max(worstFull, Math.abs(fullErr));
            }
        check("F2 se estaciona en todos los casos", allParked, "");
        check("error de estacionado (meta ±2°, máximo ±5° por R102)", worst <= 2.0, String.format("peor %.2f°", worst));
        check("modo lleno: F2 en +25°", worstFull <= 2.5, String.format("peor %.2f°", worstFull));
    }

    static void testIntakeRejectAndUnjam() {
        // NECTAR rival en la garganta
        final BallType[] seen = {BallType.NONE};
        final boolean[] mouthOn = {false};
        Fakes.Motor m = new Fakes.Motor();
        BeamSensor magnet = new BeamSensor(Fakes.digital(() -> true), false);
        BeamSensor mouth = new BeamSensor(Fakes.digital(() -> mouthOn[0]), true);
        BeamSensor throat = new BeamSensor(Fakes.digital(() -> false), true);
        BallSensor color = new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0);
        IntakeF2 in = new IntakeF2(m.proxy(), magnet, mouth, throat, color, null, Alliance.RED);
        in.setCommand(Intake.Command.IN);
        long now = 0;
        for (int i = 0; i < 10; i++) { now += 10 * MS; in.update(now, false); }
        seen[0] = BallType.NECTAR_BLUE;
        boolean filaReverse = false;
        double reverseS = 0;
        for (int i = 0; i < 100; i++) {
            now += 10 * MS;
            in.update(now, false);
            if (in.wantsFilaReverse()) { filaReverse = true; reverseS += 0.01; }
        }
        check("G408: NECTAR rival adentro -> reversa del intake y de la fila", filaReverse,
                String.format("%.2f s de reversa", reverseS));
        check("G408: se cuenta una sola vez aunque se siga viendo", in.consumeRejected() == 1, "");

        // Atasco en la boca: pelota en BB1 sin que la garganta cuente
        seen[0] = BallType.NONE;
        for (int i = 0; i < 50; i++) { now += 10 * MS; in.update(now, false); }
        mouthOn[0] = true;
        int pulses = 0;
        boolean wasReverse = false;
        for (int i = 0; i < 300; i++) {
            now += 10 * MS;
            in.update(now, false);
            boolean rev = m.power < 0;
            if (rev && !wasReverse) pulses++;
            wasReverse = rev;
        }
        check("atasco: 3 pulsos de reversa y luego aviso", pulses == 3 && in.warning().startsWith("atasco"),
                pulses + " pulsos, aviso '" + in.warning() + "'");

        // Conteo en la garganta: la 4a pelota se queda parada sobre el haz
        final boolean[] beam = {false};
        Fakes.Motor m2 = new Fakes.Motor();
        IntakeF2 in2 = new IntakeF2(m2.proxy(), new BeamSensor(Fakes.digital(() -> true), false),
                new BeamSensor(Fakes.digital(() -> false), true), new BeamSensor(Fakes.digital(() -> beam[0]), true),
                new BallSensor(out -> { fill(out, BallType.NONE, 0); return true; }, SensorCfg.THROAT, 0.0),
                null, Alliance.RED);
        in2.setCommand(Intake.Command.IN);
        int counted = 0;
        long n2 = 0;
        boolean[] pattern = new boolean[300];
        for (int i = 20; i < 40; i++) pattern[i] = true;      // pelota 1 pasa
        for (int i = 80; i < 200; i++) pattern[i] = true;     // pelota 2 se queda parada (4a) y luego avanza
        for (int i = 240; i < 260; i++) pattern[i] = true;    // pelota 3 pasa
        for (boolean b : pattern) {
            n2 += 10 * MS;
            beam[0] = b;
            in2.update(n2, false);
            counted += in2.consumeEntered();
        }
        check("garganta: 3 pelotas = 3 cuentas, aunque una se quede sobre el haz", counted == 3, counted + " cuentas");
    }
}
