import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BallType;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Config.FeederCfg;
import org.firstinspires.ftc.teamcode.Config.FlywheelCfg;
import org.firstinspires.ftc.teamcode.Config.HoodCfg;
import org.firstinspires.ftc.teamcode.Config.IntakeLenguaCfg;
import org.firstinspires.ftc.teamcode.Config.SensorCfg;
import org.firstinspires.ftc.teamcode.Config.TurretCfg;
import org.firstinspires.ftc.teamcode.Config.VisionCfg;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeLengua;
import org.firstinspires.ftc.teamcode.sensors.BallSensor;
import org.firstinspires.ftc.teamcode.sensors.BeamSensor;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
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
        System.out.println("== el piloto y la visión: quién manda en cuál CELL está arriba");
        testHiveOverride();
        System.out.println("== tiempos del Limelight");
        testVisionTiming();
        System.out.println("== torreta en lazo cerrado (Limelight simulado, retraso, chasis girando)");
        for (double speed : new double[]{140, 95, 190}) testTurretLoop(speed);
        testTurretWrap(360);
        testTurretWrap(450);
        testTurretInitSettle();
        System.out.println("== hood: ley de los dos servos");
        testHood();
        System.out.println("== flywheel: caída de rpm = tiro");
        testFlywheelDip();
        System.out.println("== alimentador: fila, copa y elevador");
        testFeeder();
        System.out.println("== alimentador: fallas");
        testFeederUpperFault();
        testFeederFlicker();
        testFeederStuckInHood();
        testFeederFlywheelOff();
        testFeederRefillLimit();
        testFeederFilaJam();
        System.out.println("== intake de la lengua: garganta, G408 y atascos");
        for (Kind k : new Kind[]{KIND_LENGUA}) {
            testIntakeRejectAndUnjam(k);
            testIntakeRivalCount(k);
            testIntakeRivalStays(k);
            testIntakeBounce(k);
        }
        System.out.println("== intake de la lengua: la lengua");
        testLenguaTongue();
        testLenguaInitBallInThroat();
        testLenguaRejectFromFolded();

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
    /*
     * La visión arranca creyendo lo del manual (fig. 10-2: arriba la roja del lado de la
     * audiencia). Para separar "decidió mal" de "no tuvo con qué decidir":
     *  A: arriba está RED_SCORING (lo contrario de lo que cree): si cambia, decidió bien.
     *  B: arriba está RED_AUDIENCE (lo que ya cree): si cambia, decidió MAL.
     */
    static void testUpDown(boolean full3d) {
        int saved = VisionCfg.UPDOWN_CONFIRM_FRAMES;
        VisionCfg.UPDOWN_CONFIRM_FRAMES = 1;
        Random rnd = new Random(11);
        int right = 0, wrong = 0, undecided = 0, total = 0, imprecise = 0;
        double worstDist = 0, worstBear = 0, worstLat = 0;
        String wrongCases = "", undecidedCases = "";
        for (double dist = 26; dist <= 130; dist += 4)
            for (double phi : new double[]{0, 20, -35, 50, -60})
                for (double aimOff : new double[]{0, 12, -18}) {
                    double[] rc = FieldSim.rowCenter(true);
                    double ax = rc[0] + dist * Math.cos(Math.toRadians(phi));
                    double ay = rc[1] + dist * Math.sin(Math.toRadians(phi));
                    double heading = Math.atan2(rc[1] - ay, rc[0] - ax) + Math.toRadians(aimOff);
                    FieldSim.Cam cam = new FieldSim.Cam(ax, ay, heading, VisionCfg.CAMERA_BEHIND_AXIS_IN,
                            VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
                    LLResult rA = FieldSim.result(cam, Cell.RED_SCORING, 1000, 5, full3d, rnd, 0.1, 0.15);
                    if (rA == null || rA.getFiducialResults().isEmpty()) continue;
                    total++;
                    CellVision vA = new CellVision(Alliance.RED);
                    CellTarget t = vA.process(rA, 1_000_000_000L);
                    if (vA.upCell() == Cell.RED_SCORING) {
                        right++;
                    } else {
                        undecided++;
                        if (undecidedCases.length() < 120) undecidedCases += String.format(" [d%.0f φ%.0f off%.0f]", dist, phi, aimOff);
                    }

                    LLResult rB = FieldSim.result(cam, Cell.RED_AUDIENCE, 1000, 5, full3d, rnd, 0.1, 0.15);
                    CellVision vB = new CellVision(Alliance.RED);
                    vB.process(rB, 1_000_000_000L);
                    if (vB.upCell() != Cell.RED_AUDIENCE) {
                        wrong++;
                        if (wrongCases.length() < 160) wrongCases += String.format(" [d%.0f φ%.0f off%.0f]", dist, phi, aimOff);
                    }

                    if (t == null || t.cell != Cell.RED_SCORING) continue;
                    if (!t.precise) { imprecise++; continue; }   // un solo tag: no fija, no se mide
                    double[] truth = FieldSim.truthApertureFromAxis(ax, ay, heading);
                    worstBear = Math.max(worstBear, Math.abs(t.bearingDeg - truth[0]));
                    worstLat = Math.max(worstLat, truth[1] * Math.abs(Math.tan(Math.toRadians(t.bearingDeg - truth[0]))));
                    if (dist >= 30 && dist <= 110) worstDist = Math.max(worstDist, Math.abs(t.distanceIn - truth[1]));
                }
        String tag = full3d ? "con Full 3D" : "sin 3D";
        check("CELL de arriba: nunca decide la equivocada (" + tag + ")", wrong == 0,
                String.format("mal %d de %d%s", wrong, total, wrongCases));
        check("CELL de arriba: decide con una foto en casi todas (" + tag + ")", right >= 0.95 * total,
                String.format("bien %d, sin decidir %d de %d%s", right, undecided, total, undecidedCases));
        check("puntería a la abertura con 2+ tags (" + tag + ")", worstLat < 1.5,
                String.format("peor %.2f in de lado (%.2f°); %d fotos de un tag (no fijan)", worstLat, worstBear, imprecise));
        check("distancia a la abertura de 30 a 110 in (" + tag + ")", worstDist < 3.0, String.format("peor %.2f in", worstDist));
        VisionCfg.UPDOWN_CONFIRM_FRAMES = saved;
    }

    static void testHiveOverride() {
        Random rnd = new Random(3);
        double[] rc = FieldSim.rowCenter(true);
        FieldSim.Cam cam = new FieldSim.Cam(rc[0] + 70, rc[1] + 10, Math.atan2(-10, -70),
                VisionCfg.CAMERA_BEHIND_AXIS_IN, VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
        CellVision v = new CellVision(Alliance.RED);
        check("arranca con la CELL de arriba de la fig. 10-2", v.upCell() == Cell.RED_AUDIENCE, "" + v.upCell());
        v.flipHive();                              // el piloto dice que se volteó; el campo dice que no
        long now = 1_000_000_000L;
        double ts = 1000;
        int switchedAt = -1;
        for (int i = 1; i <= 20; i++) {
            now += 33 * MS;
            ts += 33;
            v.process(FieldSim.result(cam, Cell.RED_AUDIENCE, ts, 5, false, rnd, 0.1, 0.1), now);
            if (v.upCell() == Cell.RED_AUDIENCE && switchedAt < 0) switchedAt = i;
        }
        check("el piloto manda hasta UPDOWN_OVERRIDE_FRAMES fotos seguidas en contra",
                switchedAt == VisionCfg.UPDOWN_OVERRIDE_FRAMES, switchedAt + " fotos");
        v.flipHive();
        for (int i = 0; i < 20; i++) {             // fotos en contra, pero sueltas
            now += 400 * MS;
            ts += 400;
            v.process(FieldSim.result(cam, Cell.RED_AUDIENCE, ts, 5, false, rnd, 0.1, 0.1), now);
        }
        check("fotos en contra separadas por más de UPDOWN_STREAK_GAP_S no se juntan", v.upCell() == Cell.RED_SCORING,
                "" + v.upCell());
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
            return FieldSim.truthApertureFromAxis(axisX, axisY, heading)[0];
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

        // Fallas a propósito
        double t = 0;
        double fireNeedS = 0.12;          // empuje que necesita la de arriba para salir
        boolean stickInHood = false;      // sale de la vista del V3 y se atora antes del flywheel (sin caída)
        BallType hood = BallType.NONE;    // la atorada entre la etapa de arriba y el flywheel
        boolean liftBlocked = false;      // la de la copa nunca sube (se queda en la copa)
        boolean liftLoses = false;        // la de la copa sale pero se queda en el tubo
        BallType tube = BallType.NONE;
        boolean copaBlocked = false;      // la fila no puede meter la pelota a la copa
        double flickerFromS = -1, flickerToS = -1;   // el V3 de arriba no la ve en este rato
        double maxCopaMm = 0, prevCopaMm = 0;
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
            f.update(now, true, false, false, bw.dip, true);
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
            f.update(now, false, false, false, bw.dip, true);
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
        bw.t += dt;
        // Copa: el servo sigue el pulso a ~236 mm/s; sin PWM se queda (abajo, en el piso)
        if (copa.pwmOn && copa.everCommanded) {
            double us = copa.us();
            double targetMm = (FeederCfg.COPA_DOWN_US - us) / (FeederCfg.COPA_DOWN_US - FeederCfg.COPA_UP_US) * FeederCfg.COPA_TRAVEL_MM;
            double s = 236 * dt;
            bw.copaMm += Math.max(-s, Math.min(s, targetMm - bw.copaMm));
        }
        boolean copaRising = bw.copaMm > bw.prevCopaMm + 1e-9;
        bw.prevCopaMm = bw.copaMm;
        bw.maxCopaMm = Math.max(bw.maxCopaMm, bw.copaMm);
        if (bw.copaMm > 57) { bw.hazard = true; bw.hazardWhy = "copa contra el tope"; }
        double ePow = elev.power;
        if (bw.copaMm > 40 && bw.upper.isBall() && bw.copa.isBall() && Math.abs(ePow) < 0.05) {
            bw.hazard = true;
            bw.hazardWhy = String.format("copa a %.0f mm con pelota arriba y motor parado", bw.copaMm);
        }
        if (copaRising && bw.copaMm > 40 && (bw.hood.isBall() || bw.tube.isBall())) {
            bw.hazard = true;
            bw.hazardWhy = String.format("copa subiendo a %.0f mm con una pelota atorada encima", bw.copaMm);
        }

        // Fila
        double fp = fila.effective() * Math.signum(FeederCfg.FILA_POWER_IN);
        if (fp > 0.5) {
            bw.filaPushS += dt;
            if (bw.filaPushS >= 0.3) {
                if (bw.fila.isBall() && !bw.copa.isBall() && bw.copaMm < 1 && !bw.copaBlocked) {
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
            if (bw.fireS >= bw.fireNeedS) {
                if (bw.stickInHood) {             // sale de la vista y se atora: sin caída de rpm
                    bw.hood = bw.upper;
                } else {
                    order.add(bw.upper);
                    bw.launched++;
                    bw.dip = true;
                }
                bw.upper = BallType.NONE;
                bw.fireS = 0;
            }
        } else if (ePow > 0.5 && bw.hood.isBall() && !bw.stickInHood) {   // el empujón saca la atorada
            bw.fireS += dt;
            if (bw.fireS >= bw.fireNeedS) {
                order.add(bw.hood);
                bw.hood = BallType.NONE;
                bw.launched++;
                bw.dip = true;
                bw.fireS = 0;
            }
        } else {
            bw.fireS = 0;
        }
        if (ePow > 0.1 && bw.copaMm >= 50 && bw.copa.isBall() && !bw.upper.isBall() && !bw.liftBlocked) {
            bw.liftS += dt;
            if (bw.liftS >= 0.25) {
                if (bw.liftLoses) bw.tube = bw.copa;
                else bw.upper = bw.copa;
                bw.copa = BallType.NONE;
                bw.liftS = 0;
            }
        } else {
            bw.liftS = 0;
        }
    }

    // =========================================================== flywheel
    static void testFlywheelDip() {
        Fakes.Motor m = new Fakes.Motor();
        Flywheel fw = new Flywheel(m.proxy());
        fw.setEnabled(true);
        double tpr = FlywheelCfg.TICKS_PER_REV;
        long now = 0;
        double target = 3000;
        fw.setTargetRpm(target);
        for (int i = 0; i < 50; i++) { now += 10 * MS; m.vel = target / 60 * tpr; fw.update(now); }
        fw.consumeShotDip();
        boolean falseDip = false;
        for (int i = 0; i < 200; i++) {          // la consigna baja 400 rpm/s; la rueda va 150 rpm atrás
            now += 10 * MS;
            target -= 4;
            fw.setTargetRpm(target);
            m.vel = (target + 150) / 60 * tpr;
            fw.update(now);
            if (fw.consumeShotDip()) falseDip = true;
        }
        check("flywheel: bajar la consigna sin pelota no es un tiro", !falseDip, "");
        for (int i = 0; i < 50; i++) { now += 10 * MS; m.vel = target / 60 * tpr; fw.update(now); fw.consumeShotDip(); }
        int dips = 0;
        for (int i = 0; i < 40; i++) {           // cae 400 rpm 50 ms y se recupera
            now += 10 * MS;
            m.vel = (i < 5 ? target - 400 : target) / 60 * tpr;
            fw.update(now);
            if (fw.consumeShotDip()) dips++;
        }
        check("flywheel: una caída de 400 rpm a velocidad es un tiro, una vez", dips == 1, dips + " caídas");
    }

    // =========================================================== alimentador: fallas
    static final class FeederRig {
        final BallWorld bw = new BallWorld();
        final Fakes.ContServo fila = new Fakes.ContServo();
        final Fakes.PosServo copa = new Fakes.PosServo();
        final Fakes.Motor elev = new Fakes.Motor();
        final ArrayList<BallType> order = new ArrayList<>();
        boolean upperFault = false;
        final Feeder f;
        long now = 0;

        FeederRig() {
            BallSensor upperS = new BallSensor(out -> {
                if (upperFault) {                 // I2C caído: el SDK da ceros y 152.4 mm
                    out[0] = out[1] = out[2] = 0;
                    out[3] = 152.4;
                    return false;
                }
                boolean flicker = bw.t >= bw.flickerFromS && bw.t < bw.flickerToS;
                fill(out, flicker ? BallType.NONE : bw.upper, 20);
                return true;
            }, SensorCfg.UPPER, 0.04);
            BallSensor copaS = new BallSensor(out -> {
                fill(out, bw.copaMm < 10 ? bw.copa : BallType.NONE, bw.copa.isNectar() ? 4.6 : 16.1);
                return true;
            }, SensorCfg.COPA, 0.04);
            BeamSensor filaB = new BeamSensor(Fakes.digital(() -> bw.fila.isBall()), true);
            f = new Feeder(fila.proxy(), copa.proxy(), elev.proxy(), upperS, copaS, filaB);
            f.initHardware(now);
        }

        /** 10 ms. Regresa los tiros contados en este paso. */
        int step(boolean intakeFeeding, boolean flywheelOn) {
            now += 10 * MS;
            f.update(now, intakeFeeding, false, false, bw.dip, flywheelOn);
            bw.dip = false;
            int shots = f.consumeShots();
            physics(bw, fila, copa, elev, 0.01, order);
            return shots;
        }
    }

    static void testFeederUpperFault() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.POLLEN;
        r.bw.copa = BallType.NECTAR_RED;
        r.upperFault = true;
        for (int i = 0; i < 600; i++) r.step(true, true);
        check("V3 de arriba sin lectura: la copa no sube (C1)", r.bw.maxCopaMm < 1 && !r.bw.hazard,
                String.format("copa hasta %.1f mm %s", r.bw.maxCopaMm, r.bw.hazardWhy));
        check("V3 de arriba sin lectura: cuenta como pelota arriba", r.f.hasUpperBall(), "");
        r.f.requestShot();
        int shots = 0;
        for (int i = 0; i < 300; i++) shots += r.step(true, true);
        check("V3 de arriba sin lectura: el tiro se confirma con la caída de rpm", shots == 1 && r.bw.launched == 1,
                shots + " contados, " + r.bw.launched + " salieron");
        check("V3 de arriba sin lectura: después del tiro tampoco sube la copa", r.bw.maxCopaMm < 1 && !r.bw.hazard,
                String.format("copa hasta %.1f mm", r.bw.maxCopaMm));
    }

    static void testFeederFlicker() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.NECTAR_RED;
        r.bw.fireNeedS = 0.6;                     // patina: tarda en salir
        r.bw.flickerFromS = 0.10;                 // y el V3 deja de verla 60 ms
        r.bw.flickerToS = 0.16;
        r.f.requestShot();
        int shots = 0;
        for (int i = 0; i < 200; i++) shots += r.step(false, true);
        check("un parpadeo del V3 a medio tiro no cuenta un tiro (C2)", shots == 1 && r.bw.launched == 1,
                shots + " contados, " + r.bw.launched + " salieron");
    }

    static void testFeederStuckInHood() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.NECTAR_RED;
        r.bw.copa = BallType.POLLEN;
        r.bw.stickInHood = true;
        r.f.requestShot();
        int shots = 0;
        boolean gapKept = true;
        for (int i = 0; i < 300; i++) {
            shots += r.step(true, true);
            if (r.bw.hood.isBall() && r.f.nextBall() != BallType.NECTAR_RED) gapKept = false;
        }
        check("atorada antes del flywheel (sin caída): no se cuenta (C2)", shots == 0 && r.bw.hood.isBall(), shots + " contados");
        check("atorada: no se rellena mientras no se confirme", r.bw.maxCopaMm < 1 && r.f.isClearPending() && !r.bw.hazard,
                String.format("copa hasta %.1f mm %s", r.bw.maxCopaMm, r.bw.hazardWhy));
        check("atorada: el gap se queda en el de la atorada (NECTAR)", gapKept, "");
        r.bw.stickInHood = false;                 // el artillero la empuja y sale
        r.f.forceShot();
        for (int i = 0; i < 100; i++) shots += r.step(true, true);
        check("atorada: el empujón la saca y se cuenta", shots == 1 && !r.f.isClearPending(),
                shots + " contados, pendiente " + r.f.isClearPending());
        for (int i = 0; i < 300; i++) r.step(true, true);
        check("atorada: después se vuelve a rellenar", r.bw.upper == BallType.POLLEN && !r.bw.hazard,
                "arriba " + r.bw.upper);

        FeederRig q = new FeederRig();            // o el artillero dice que ya está libre
        q.bw.upper = BallType.POLLEN;
        q.bw.copa = BallType.NECTAR_RED;
        q.bw.stickInHood = true;
        q.f.requestShot();
        for (int i = 0; i < 200; i++) q.step(true, true);
        q.bw.hood = BallType.NONE;                // la sacó a mano
        q.f.release();
        for (int i = 0; i < 300; i++) q.step(true, true);
        check("atorada: con B (libre) se vuelve a rellenar", q.bw.upper == BallType.NECTAR_RED, "arriba " + q.bw.upper);
    }

    static void testFeederFlywheelOff() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.POLLEN;
        r.bw.fireNeedS = 0.5;
        r.f.requestShot();
        for (int i = 0; i < 6; i++) r.step(false, true);
        double before = r.elev.power;
        r.step(false, false);                     // se apaga el flywheel a medio tiro
        check("flywheel apagado a medio tiro: el elevador se detiene", before > 0.5 && Math.abs(r.elev.power) < 0.01
                && r.f.getState() == Feeder.State.IDLE, String.format("antes %.2f, después %.2f", before, r.elev.power));
    }

    static void testFeederRefillLimit() {
        FeederRig r = new FeederRig();
        r.bw.copa = BallType.POLLEN;
        r.bw.liftBlocked = true;                  // la pelota nunca llega arriba
        int rises = 0;
        Feeder.State prev = r.f.getState();
        for (int i = 0; i < 1500; i++) {
            r.step(false, true);
            if (r.f.getState() == Feeder.State.COPA_UP && prev != Feeder.State.COPA_UP) rises++;
            prev = r.f.getState();
        }
        check("rellenar que falla se detiene tras REFILL_MAX_FAILS intentos", rises == FeederCfg.REFILL_MAX_FAILS
                && r.f.isRefillBlocked(), rises + " subidas");

        FeederRig q = new FeederRig();
        q.bw.copa = BallType.POLLEN;
        q.bw.fila = BallType.NECTAR_RED;
        q.bw.liftLoses = true;                    // sale de la copa y se queda en el tubo
        rises = 0;
        prev = q.f.getState();
        for (int i = 0; i < 1500; i++) {
            q.step(true, true);
            if (q.f.getState() == Feeder.State.COPA_UP && prev != Feeder.State.COPA_UP) rises++;
            prev = q.f.getState();
        }
        check("si la copa regresa vacía de un rellenado fallido, se detiene a la primera", rises == 1
                && q.f.isRefillBlocked() && !q.bw.hazard, rises + " subidas " + q.bw.hazardWhy);
    }

    static void testFeederFilaJam() {
        FeederRig r = new FeederRig();
        r.bw.fila = BallType.POLLEN;
        r.bw.copaBlocked = true;                  // la copa no la recibe
        double pushLate = 0;
        for (int i = 0; i < 2000; i++) {
            r.step(false, true);
            if (r.bw.t >= 4 && r.fila.effective() * Math.signum(FeederCfg.FILA_POWER_IN) > 0.5) pushLate += 0.01;
        }
        check("fila atorada: se apaga y solo da un empujón por espera (M9)", pushLate / 16 < 0.35,
                String.format("empuja %.0f %% del tiempo después de los intentos", 100 * pushLate / 16));
    }

    // =========================================================== intake (cualquiera que implemente Intake)
    interface IntakeMaker {
        Intake make(Fakes.Motor m, BeamSensor mouth, BeamSensor throat, BallSensor color);
    }

    /** Un intake para las pruebas que valen para los dos (misma garganta, mismo G408). */
    static final class Kind {
        final String name;
        final double rejectMaxS;
        final IntakeMaker mk;

        Kind(String name, double rejectMaxS, IntakeMaker mk) {
            this.name = name;
            this.rejectMaxS = rejectMaxS;
            this.mk = mk;
        }
    }

    static final Kind KIND_LENGUA = new Kind("lengua", IntakeLenguaCfg.REJECT_MAX_S, (m, mouth, throat, color) ->
            new IntakeLengua(m.proxy(), new Fakes.PosServo().proxy(), mouth, throat, color, null, Alliance.RED));

    /** Lo deja jalando (la lengua baja). Regresa el reloj. */
    static long warmUp(Intake in, long now) {
        in.setCommand(Intake.Command.IN);
        for (int i = 0; i < 80; i++) { now += 10 * MS; in.update(now, false); }
        in.consumeEntered();
        in.consumeRejected();
        return now;
    }

    static void testIntakeRejectAndUnjam(Kind k) {
        final BallType[] seen = {BallType.NONE};
        final boolean[] mouthOn = {false};
        Fakes.Motor m = new Fakes.Motor();
        Intake in = k.mk.make(m, new BeamSensor(Fakes.digital(() -> mouthOn[0]), true),
                new BeamSensor(Fakes.digital(() -> false), true),
                new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0));
        long now = warmUp(in, 0);
        seen[0] = BallType.NECTAR_BLUE;           // NECTAR rival en la garganta
        boolean filaReverse = false;
        double reverseS = 0;
        for (int i = 0; i < 100; i++) {
            now += 10 * MS;
            in.update(now, false);
            if (in.wantsFilaReverse()) { filaReverse = true; reverseS += 0.01; }
        }
        check(k.name + ": G408 rival adentro -> reversa del intake y de la fila", filaReverse && m.power < 0,
                String.format("%.2f s de reversa", reverseS));
        check(k.name + ": G408 una rival que nunca se sumó no se resta",
                in.consumeRejected() == 0 && in.consumeEntered() == 0, "");

        seen[0] = BallType.NONE;                  // atasco: pelota en BB1 sin que la garganta cuente
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
        check(k.name + ": atasco -> 3 pulsos de reversa y luego aviso", pulses == 3 && in.warning().startsWith("atasco"),
                pulses + " pulsos, aviso '" + in.warning() + "'");

        final boolean[] beam = {false};           // la 4a pelota se queda parada sobre el haz
        Fakes.Motor m2 = new Fakes.Motor();
        Intake in2 = k.mk.make(m2, new BeamSensor(Fakes.digital(() -> false), true),
                new BeamSensor(Fakes.digital(() -> beam[0]), true),
                new BallSensor(out -> { fill(out, BallType.NONE, 0); return true; }, SensorCfg.THROAT, 0.0));
        long n2 = warmUp(in2, 0);
        int counted = 0;
        boolean[] pattern = new boolean[300];
        for (int i = 20; i < 40; i++) pattern[i] = true;      // pelota 1 pasa
        for (int i = 80; i < 200; i++) pattern[i] = true;     // pelota 2 se queda parada y luego avanza
        for (int i = 240; i < 260; i++) pattern[i] = true;    // pelota 3 pasa
        for (boolean b : pattern) {
            n2 += 10 * MS;
            beam[0] = b;
            in2.update(n2, false);
            counted += in2.consumeEntered();
        }
        check(k.name + ": garganta 3 pelotas = 3 cuentas, aunque una se quede sobre el haz", counted == 3, counted + " cuentas");
    }

    static void testIntakeRivalCount(Kind k) {
        final boolean[] beam = {false};
        final BallType[] seen = {BallType.NONE};
        Fakes.Motor m = new Fakes.Motor();
        Intake in = k.mk.make(m, new BeamSensor(Fakes.digital(() -> false), true),
                new BeamSensor(Fakes.digital(() -> beam[0]), true),
                new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0));
        long now = warmUp(in, 0);
        beam[0] = true;                           // llega a la garganta y se cuenta...
        for (int i = 0; i < 6; i++) { now += 10 * MS; in.update(now, false); }
        seen[0] = BallType.NECTAR_BLUE;           // ...y luego el V3 ve que es rival
        for (int i = 0; i < 20; i++) { now += 10 * MS; in.update(now, false); }
        beam[0] = false;
        seen[0] = BallType.NONE;
        for (int i = 0; i < 60; i++) { now += 10 * MS; in.update(now, false); }
        int entered = in.consumeEntered(), rejected = in.consumeRejected();
        check(k.name + ": G408 una rival que ya se sumó se resta (cuenta neta 0)", entered == 1 && rejected == 1,
                entered + " entraron, " + rejected + " rechazadas");

        beam[0] = true;                           // el V3 la ve antes de los 30 ms del haz
        seen[0] = BallType.NECTAR_BLUE;
        for (int i = 0; i < 30; i++) { now += 10 * MS; in.update(now, false); }
        beam[0] = false;
        seen[0] = BallType.NONE;
        for (int i = 0; i < 60; i++) { now += 10 * MS; in.update(now, false); }
        entered = in.consumeEntered();
        rejected = in.consumeRejected();
        check(k.name + ": G408 una rival vista antes de contarse ni se suma ni se resta", entered == 0 && rejected == 0,
                entered + " entraron, " + rejected + " rechazadas");
    }

    static void testIntakeRivalStays(Kind k) {
        final BallType[] seen = {BallType.NONE};
        Fakes.Motor m = new Fakes.Motor();
        Intake in = k.mk.make(m, new BeamSensor(Fakes.digital(() -> false), true),
                new BeamSensor(Fakes.digital(() -> false), true),
                new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0));
        long now = warmUp(in, 0);
        seen[0] = BallType.NECTAR_BLUE;
        boolean pulled = false;
        double holdAt = -1;
        for (int i = 1; i <= 300; i++) {
            now += 10 * MS;
            in.update(now, false);
            if (m.power > 0) pulled = true;
            if (in.holdsFila() && holdAt < 0) holdAt = i * 0.01;
        }
        check(k.name + ": G408 si la rival se queda, nunca se vuelve a jalar", !pulled, "");
        check(k.name + ": G408 tras REJECT_MAX_S se detiene y detiene la fila", Math.abs(holdAt - k.rejectMaxS) <= 0.05
                && m.power == 0, String.format("se detuvo a %.2f s", holdAt));
        seen[0] = BallType.NONE;
        for (int i = 0; i < 30; i++) { now += 10 * MS; in.update(now, false); }
        check(k.name + ": G408 cuando se va, vuelve a jalar", m.power > 0 && !in.holdsFila(), "");
    }

    static void testIntakeBounce(Kind k) {
        final boolean[] beam = {false};
        Fakes.Motor m = new Fakes.Motor();
        Intake in = k.mk.make(m, new BeamSensor(Fakes.digital(() -> false), true),
                new BeamSensor(Fakes.digital(() -> beam[0]), true),
                new BallSensor(out -> { fill(out, BallType.NONE, 0); return true; }, SensorCfg.THROAT, 0.0));
        long now = warmUp(in, 0);
        boolean[] pattern = new boolean[60];
        for (int i = 5; i < 10; i++) pattern[i] = true;     // pelota 1: 50 ms
        for (int i = 12; i < 17; i++) pattern[i] = true;    // rebota tras 20 ms libre
        for (int i = 35; i < 40; i++) pattern[i] = true;    // pelota 2, 180 ms después
        int counted = 0;
        for (boolean b : pattern) {
            now += 10 * MS;
            beam[0] = b;
            in.update(now, false);
            counted += in.consumeEntered();
        }
        check(k.name + ": garganta: un rebote no cuenta otra pelota", counted == 2, counted + " cuentas");
    }

    // =========================================================== intake de la lengua
    static boolean near(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }

    /** Posiciones de prueba (las reales las da el CAD): con las de Config el servo no se mueve. */
    /** Posiciones de prueba (las reales las da el CAD): con las de Config el servo no se mueve. */
    static double[] setTestTonguePositions() {
        double[] saved = {IntakeLenguaCfg.FOLDED_US, IntakeLenguaCfg.DOWN_US};
        IntakeLenguaCfg.FOLDED_US = 1000;
        IntakeLenguaCfg.DOWN_US = 2000;
        return saved;
    }

    static void restoreTonguePositions(double[] saved) {
        IntakeLenguaCfg.FOLDED_US = saved[0];
        IntakeLenguaCfg.DOWN_US = saved[1];
    }

    static IntakeLengua lengua(Fakes.Motor m, Fakes.PosServo sv, boolean[] throatBeam, BallType[] seen) {
        return new IntakeLengua(m.proxy(), sv.proxy(), new BeamSensor(Fakes.digital(() -> false), true),
                new BeamSensor(Fakes.digital(() -> throatBeam[0]), true),
                new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0),
                null, Alliance.RED);
    }

    static void testLenguaTongue() {
        double[] saved = setTestTonguePositions();
        try {
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            boolean[] beam = {false};
            IntakeLengua in = lengua(m, sv, beam, new BallType[]{BallType.NONE});
            double travel = IntakeLenguaCfg.TRAVEL_S;
            long now = 0;
            double readyAt = -1;
            for (int i = 1; i <= 100; i++) {
                now += 10 * MS;
                if (in.initStep(now) && readyAt < 0) readyAt = i * 0.01;
            }
            check("lengua: en INIT va a plegada y luego se suelta (PWM apagado)", in.isStartReady() && !sv.pwmOn
                    && near(sv.position, IntakeLengua.positionFor(IntakeLengua.Tongue.FOLDED)) && m.power == 0
                    && Math.abs(readyAt - travel) <= 0.02, String.format("lista a %.2f s (viaje %.2f s)", readyAt, travel));

            in.setCommand(Intake.Command.IN);         // Play con 4 pelotas e intake prendido
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.update(now, true); }
            check("lengua: al dar Play con 4 pelotas se queda plegada y suelta",
                    in.getTongue() == IntakeLengua.Tongue.FOLDED && !sv.pwmOn && m.power == 0, "" + in.getTongue());

            double pullAt = -1;                       // ya hay lugar: baja y luego jala
            for (int i = 1; i <= 100; i++) {
                now += 10 * MS;
                in.update(now, false);
                if (m.power > 0 && pullAt < 0) pullAt = i * 0.01;
            }
            check("lengua: con lugar e intake prendido baja, y M_F jala hasta que llegó",
                    in.getTongue() == IntakeLengua.Tongue.DOWN && sv.pwmOn
                            && near(sv.position, IntakeLengua.positionFor(IntakeLengua.Tongue.DOWN))
                            && Math.abs(pullAt - travel) <= 0.02, String.format("jala a %.2f s", pullAt));
            check("lengua: jalando, la fila puede traer la siguiente", in.isFeeding(), "");

            beam[0] = true;                           // llega la 4a y se queda en la garganta
            boolean moved = false;
            for (int i = 0; i < 200; i++) {
                now += 10 * MS;
                in.update(now, true);
                if (in.getTongue() != IntakeLengua.Tongue.DOWN
                        || !near(sv.position, IntakeLengua.positionFor(IntakeLengua.Tongue.DOWN))) moved = true;
            }
            check("lengua: con 4 pelotas se queda ABAJO y M_F para en BRAKE (no empuja la 4a)", !moved && sv.pwmOn
                    && m.power == 0 && m.zpb == DcMotor.ZeroPowerBehavior.BRAKE, "" + in.getTongue());

            beam[0] = false;                          // salió un tiro
            in.setCommand(Intake.Command.STOP);
            for (int i = 0; i < 50; i++) { now += 10 * MS; in.update(now, false); }
            check("lengua: con lugar pero intake apagado se queda abajo y quieta",
                    in.getTongue() == IntakeLengua.Tongue.DOWN && m.power == 0, "" + in.getTongue());
            in.setCommand(Intake.Command.IN);
            for (int i = 0; i < 20; i++) { now += 10 * MS; in.update(now, false); }
            check("lengua: al prender el intake jala de inmediato (ya está abajo)", m.power > 0, "");
        } finally {
            restoreTonguePositions(saved);
        }
    }

    static void testLenguaInitBallInThroat() {
        double[] saved = setTestTonguePositions();
        try {
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            boolean[] beam = {true};                  // una pelota en la garganta al dar INIT
            IntakeLengua in = lengua(m, sv, beam, new BallType[]{BallType.POLLEN});
            long now = 0;
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            check("lengua: con pelota en la garganta no se pliega en INIT y avisa", !sv.everCommanded && !in.isStartReady()
                    && in.warning().contains("garganta"), "aviso '" + in.warning() + "'");
            beam[0] = false;                          // la sacan
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            check("lengua: ya sin pelota, se pliega y queda lista", in.isStartReady() && !sv.pwmOn
                    && near(sv.position, IntakeLengua.positionFor(IntakeLengua.Tongue.FOLDED)), "");
        } finally {
            restoreTonguePositions(saved);
        }
    }

    static void testLenguaRejectFromFolded() {
        double[] saved = setTestTonguePositions();
        try {
            BallType[] seen = {BallType.NONE};
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            IntakeLengua in = lengua(m, sv, new boolean[]{false}, seen);
            long now = 0;
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            in.setCommand(Intake.Command.IN);
            for (int i = 0; i < 20; i++) { now += 10 * MS; in.update(now, true); }   // lleno: sigue plegada
            seen[0] = BallType.NECTAR_BLUE;           // una rival en la garganta con la lengua plegada
            boolean badWhileMoving = false;
            double reverseAt = -1;
            for (int i = 1; i <= 120; i++) {
                now += 10 * MS;
                in.update(now, true);
                boolean down = in.getTongue() == IntakeLengua.Tongue.DOWN && in.isTongueSettled();
                if (!down && (m.power != 0 || in.wantsFilaReverse() || !in.holdsFila())) badWhileMoving = true;
                if (m.power < 0 && in.wantsFilaReverse() && reverseAt < 0) reverseAt = i * 0.01;
            }
            check("lengua: rival con la lengua plegada -> primero baja, con M_F y la fila quietos", !badWhileMoving, "");
            check("lengua: ya abajo, reversa de M_F y de la fila",
                    Math.abs(reverseAt - IntakeLenguaCfg.TRAVEL_S) <= 0.02, String.format("reversa a %.2f s", reverseAt));
        } finally {
            restoreTonguePositions(saved);
        }
    }
}
