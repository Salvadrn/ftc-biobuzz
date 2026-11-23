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
import org.firstinspires.ftc.teamcode.feeder.BallInventory;
import org.firstinspires.ftc.teamcode.feeder.Feeder;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeLengua;
import org.firstinspires.ftc.teamcode.intake.TongueServo;
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
        testHoodServoSigns();
        testHoodFrozenStart();
        testHoodStops();
        System.out.println("== flywheel: caída de rpm = tiro");
        testFlywheelDip();
        testFlywheelCalcDips();
        System.out.println("== alimentador: fila, copa y elevador");
        testFeeder();
        System.out.println("== alimentador: fallas");
        testFeederUpperFault();
        testFeederFlicker();
        testFeederStuckInHood();
        testFeederFlywheelOff();
        testFeederRefillLimit();
        testFeederFilaJam();
        testFeederCalcDipUnconfirmed();
        System.out.println("== alimentador: V3 de arriba, subida de la copa y fin de LIFT");
        testFeederLiftEnd();
        testFeederLiftNoise();
        testFeederTopFlicker();
        testFeederCopaUpOwnBall();
        testFeederNectarStuckInCopa();
        testFeederSeatThird();
        System.out.println("== intake de la lengua: garganta, G408 y atascos");
        boolean savedCal = calibrate();            // un robot ya calibrado, con los µs de cálculo de Config
        try {
            for (Kind k : new Kind[]{KIND_LENGUA}) {
                testIntakeRejectAndUnjam(k);
                testIntakeRivalCount(k);
                testIntakeRivalStays(k);
                testIntakeBounce(k);
            }
            System.out.println("== intake de la lengua: la lengua");
            testLenguaTongue();
            testLenguaInitBallInThroat();
            testLenguaInitBallAtMouth();
            testLenguaRejectFromFolded();
            System.out.println("== intake de la lengua: 4a pelota, sobrecorriente y sin calibrar");
            testLenguaFourthAtMouth();
            testLenguaOverCurrent();
        } finally {
            restoreCalibrated(savedCal);
        }
        testLenguaUncalibrated();

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

    /*
     * runHood usa la convención de Config para mover los servos "de verdad", así que no puede
     * ver un signo mal puesto. Aquí los servos van como en el CAD (documento v138, 4.1): el de
     * gap gira el eslabón +0.15° por µs y el de ángulo, montado al revés, -0.15° por µs.
     * Regresa {margen mínimo a la Limelight, a las FlatTop, llegó a NECTAR, listo con NECTAR}.
     */
    static double[] runHoodCad(double angleUsPerDeg, boolean faceOpposite) {
        double savedA = HoodCfg.ANGLE_US_PER_DEG;
        boolean savedF = HoodCfg.SERVOS_FACE_OPPOSITE;
        HoodCfg.ANGLE_US_PER_DEG = angleUsPerDeg;
        HoodCfg.SERVOS_FACE_OPPOSITE = faceOpposite;
        try {
            Fakes.PosServo gs = new Fakes.PosServo(), as = new Fakes.PosServo();
            Hood hood = new Hood(gs.proxy(), as.proxy(), new StateStore.Memory());
            long now = 0;
            hood.start(now);
            double minLl = 99, minFt = 99;
            boolean reached = false, readyForNectar = false;
            BallType[] plan = {BallType.POLLEN, BallType.NECTAR_RED, BallType.POLLEN, BallType.NECTAR_BLUE};
            for (BallType b : plan) {
                hood.setGapFor(b);
                hood.setDelta(0.0);
                for (int i = 0; i < 300; i++) {
                    now += 5 * MS;
                    hood.update(now);
                    double gap = (gs.us() - 1500) * 0.15;
                    double delta = gap + (as.us() - 1500) * -0.15;
                    minLl = Math.min(minLl, delta - limelightLimit(gap));
                    minFt = Math.min(minFt, flatTopLimit(gap) - delta);
                    if (b.needsNectarGap() && Math.abs(gap - HoodCfg.GAP_NECTAR_DEG) < 0.2) reached = true;
                    if (b.needsNectarGap() && hood.isReady(now)) readyForNectar = true;
                }
            }
            return new double[]{minLl, minFt, reached ? 1 : 0, readyForNectar ? 1 : 0};
        } finally {
            HoodCfg.ANGLE_US_PER_DEG = savedA;
            HoodCfg.SERVOS_FACE_OPPOSITE = savedF;
        }
    }

    static void testHoodServoSigns() {
        check("hood: los signos de Config cuadran con el armado del CAD", Hood.signsConsistent(), "");
        double[] ok = runHoodCad(HoodCfg.ANGLE_US_PER_DEG, true);
        check("hood con servos como en el CAD: va a NECTAR y δ se queda en la envolvente",
                ok[2] == 1 && ok[3] == 1 && ok[0] > 8 && ok[1] > 20,
                String.format("margen %.1f° a la Limelight, %.1f° a las FlatTop", ok[0], ok[1]));
        double[] blocked = runHoodCad(1.0 / 0.15, true);
        check("hood: con el ángulo del mismo signo que el gap, el seguro no va a NECTAR (ni da listo)",
                blocked[2] == 0 && blocked[3] == 0 && blocked[0] > 8 && blocked[1] > 20,
                String.format("margen %.1f° / %.1f°", blocked[0], blocked[1]));
        double[] forced = runHoodCad(1.0 / 0.15, false);   // sin el seguro: como si el armado fuera otro
        check("hood: sin el seguro, con +6.667 el modelo del CAD choca con la Limelight", forced[0] < 0,
                String.format("margen %.1f° a la Limelight", forced[0]));
    }

    /**
     * Topes reales del gap (documento del CAD v138, 4.2): el eslabón para en +0.91° y en
     * -35.18°, los µs se toman en los topes y ANGLE_ZERO_US con δ 0 en el tope de POLLEN.
     * Regresa el δ real en NECTAR con δ 0.
     */
    static double hoodDeltaAtNectarStop(double gapNectarDeg) {
        double savedDeg = HoodCfg.GAP_NECTAR_DEG, savedUs = HoodCfg.GAP_NECTAR_US;
        HoodCfg.GAP_NECTAR_DEG = gapNectarDeg;
        HoodCfg.GAP_NECTAR_US = HoodCfg.GAP_POLLEN_US - (0.91 + 35.18) / 0.15;   // los µs del tope de NECTAR
        try {
            Fakes.PosServo gs = new Fakes.PosServo(), as = new Fakes.PosServo();
            Hood hood = new Hood(gs.proxy(), as.proxy(), new StateStore.Memory());
            long now = 0;
            hood.start(now);
            hood.setGapFor(BallType.NECTAR_RED);
            hood.setDelta(0.0);
            for (int i = 0; i < 400; i++) {
                now += 5 * MS;
                hood.update(now);
            }
            double link = 0.91 + (gs.us() - HoodCfg.GAP_POLLEN_US) * 0.15;
            double angle = -0.91 + (as.us() - HoodCfg.ANGLE_ZERO_US) * -0.15;
            return link + angle;
        } finally {
            HoodCfg.GAP_NECTAR_DEG = savedDeg;
            HoodCfg.GAP_NECTAR_US = savedUs;
        }
    }

    static void testHoodFrozenStart() {
        double savedA = HoodCfg.ANGLE_US_PER_DEG;
        try {
            StateStore store = new StateStore.Memory();
            store.putDouble("hood.gap", HoodCfg.GAP_NECTAR_DEG);   // quedó en NECTAR...
            store.putDouble("hood.delta", 0);
            HoodCfg.ANGLE_US_PER_DEG = 1.0 / 0.15;                 // ...y Config ya no cuadra
            Fakes.PosServo gs = new Fakes.PosServo(), as = new Fakes.PosServo();
            Hood hood = new Hood(gs.proxy(), as.proxy(), store);
            long now = 0;
            hood.start(now);
            hood.setGapFor(BallType.POLLEN);
            for (int i = 0; i < 100; i++) { now += 10 * MS; hood.update(now); }
            check("hood guardado en NECTAR con los signos sin cuadrar: se queda sin PWM y no da listo",
                    hood.isFrozen() && !gs.everCommanded && !as.everCommanded && !hood.isReady(now), "");
            hood.assumeAtPollen(now);              // lo pusieron a mano en POLLEN
            for (int i = 0; i < 100; i++) { now += 10 * MS; hood.update(now); }
            check("hood: confirmado a mano en POLLEN, vuelve a moverse y queda listo",
                    !hood.isFrozen() && gs.everCommanded && hood.isReady(now) && hood.getGapDeg() == HoodCfg.GAP_POLLEN_DEG, "");
        } finally {
            HoodCfg.ANGLE_US_PER_DEG = savedA;
        }
    }

    static void testHoodStops() {
        double old = hoodDeltaAtNectarStop(-34.3), calc = hoodDeltaAtNectarStop(-36.1);
        check("hood, topes reales: con GAP_NECTAR_DEG -34.3 el δ real en NECTAR da -1.8°", Math.abs(old + 1.8) < 0.1,
                String.format("%.2f°", old));
        check("hood, topes reales: con -36.1 da 0 ± 0.1°", Math.abs(calc) <= 0.1, String.format("%.2f°", calc));
        double saved = HoodCfg.GAP_NECTAR_DEG;
        boolean follows = true;
        for (double g : new double[]{-34.3, -35.2, -36.1}) {
            HoodCfg.GAP_NECTAR_DEG = g;
            follows &= Math.abs(Hood.angleMaxDeg() - (-g + HoodCfg.DELTA_SAFE_MAX_DEG)) < 1e-9;
        }
        HoodCfg.GAP_NECTAR_DEG = saved;
        check("hood: el recorrido del servo de ángulo sigue a GAP_NECTAR_DEG", follows,
                String.format("hoy de %.1f° a %.1f°", Hood.angleMinDeg(), Hood.angleMaxDeg()));
    }

    // =========================================================== alimentador
    /*
     * Geometría del tubo (CAD v131 y documento del CAD v138, 4.7): lente del V3 de arriba en
     * y 34.5, z 128, mirando a -Y; eje del tubo en y -20 (54.5 mm); aro de la copa (ID 40) en
     * z 6..11.5. La distancia que lee el V3 de arriba es la geométrica (lo que lee fuera de su
     * eje está sin medir: se prueba con conos de 30° y 45°).
     */
    static final double LENS_Z = 128, LENS_TO_AXIS = 54.5;
    /** CALC del documento: 0.48 m/s a ELEVATOR_STAGE_POWER 0.25, sin carga. */
    static final double LIFT_MM_S_PER_POWER = 1920;

    static double radius(BallType b) { return b == BallType.POLLEN ? 35.55 : 45.975; }

    /** Centro de una pelota sentada en el aro de la copa. */
    static double copaCenterZ(BallType b, double copaMm) {
        double r = radius(b);
        return copaMm + 11.5 + Math.sqrt(r * r - 20 * 20);
    }

    /** Altura del centro a la que la pelota toca el flywheel (CAD, D106). */
    static double flywheelContactZ(BallType b) { return b == BallType.POLLEN ? 170.8 : 156.5; }

    /**
     * Hasta dónde la sostienen las Gecko de z 128 (CAD v146, D131): más arriba ya no las toca y
     * la cuña de las locas con resorte la puede empujar hacia el flywheel.
     */
    static double geckoHoldZ(BallType b) { return b == BallType.POLLEN ? 145.4 : 147.0; }

    static final class BallWorld {
        BallType throat = BallType.NONE, fila = BallType.NONE, copa = BallType.NONE, upper = BallType.NONE;
        double copaMm = 0;
        double filaPushS = 0, fireS = 0;
        int launched = 0;
        boolean hazard = false;
        String hazardWhy = "";
        boolean dip = false;

        // La pelota que sube de la copa a su etapa, y dónde quedó la de arriba (NaN = z 150)
        BallType lifting = BallType.NONE;
        double liftZ = 0, upperZ = Double.NaN;
        double coneDeg = 30;              // el V3 de arriba ve hasta este ángulo de su eje
        double upperSampleT = -1, upperSampleMm = 200;
        BallType upperSampleType = BallType.NONE;
        boolean encoderDead = false;      // el encoder del elevador no está conectado
        double ballSpeedRatio = 1.0;      // la pelota sube a esta fracción de la superficie de las Gecko
        double upperNoiseMm = 0;          // ruido (desviación) de la distancia del V3 de arriba
        double upperFloorMm = 0;          // el V3 de arriba nunca marca menos que esto (curva más plana)
        Random noise = new Random(1);

        // Fallas a propósito
        double t = 0;
        double fireNeedS = 0.12;          // empuje que necesita la de arriba para salir
        boolean stickInHood = false;      // sale de la vista del V3 y se atora antes del flywheel (sin caída)
        BallType hood = BallType.NONE;    // la atorada entre la etapa de arriba y el flywheel
        boolean liftBlocked = false;      // la de la copa nunca sube (se queda en la copa)
        boolean liftLoses = false;        // la de la copa sale pero se queda en el tubo (abajo del V3)
        BallType tube = BallType.NONE;
        boolean copaBlocked = false;      // la fila no puede meter la pelota a la copa
        double flickerFromS = -1, flickerToS = -1;   // el V3 de arriba no la ve en este rato
        double maxCopaMm = 0, prevCopaMm = 0;
        double minTopGapMm = 99;          // lo más cerca que quedó la de arriba (quieta) de la de la copa

        double upperZOrDefault() { return Double.isNaN(upperZ) ? 150 : upperZ; }
    }

    /**
     * Lo que lee el V3 de arriba: la pelota más cercana dentro de su cono (la que sube, la de
     * arriba o la de la copa). Mide cada 12.5 ms (SensorCfg.UPPER_FAST_PROXIMITY).
     */
    static void upperReading(BallWorld bw, double[] out) {
        if (bw.upperSampleT < 0 || bw.t - bw.upperSampleT >= 0.0125 - 1e-9) {
            bw.upperSampleT = bw.t;
            BallType seen = BallType.NONE;
            double best = 200;
            BallType[] balls = {bw.lifting, bw.upper, bw.copa};
            double[] zs = {bw.liftZ, bw.upperZOrDefault(), copaCenterZ(bw.copa, bw.copaMm)};
            for (int i = 0; i < balls.length; i++) {
                if (!balls[i].isBall()) continue;
                double dz = Math.abs(zs[i] - LENS_Z);
                if (Math.toDegrees(Math.atan2(dz, LENS_TO_AXIS)) > bw.coneDeg) continue;
                double d = Math.max(bw.upperFloorMm, Math.hypot(LENS_TO_AXIS, dz) - radius(balls[i]))
                        + bw.upperNoiseMm * bw.noise.nextGaussian();
                if (d < best) {
                    best = d;
                    seen = balls[i];
                }
            }
            bw.upperSampleType = seen;
            bw.upperSampleMm = best;
        }
        boolean flicker = bw.t >= bw.flickerFromS && bw.t < bw.flickerToS;
        fill(out, flicker ? BallType.NONE : bw.upperSampleType, bw.upperSampleMm);
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
            upperReading(bw, out);
            return true;
        }, SensorCfg.UPPER, 0.04);
        BallSensor copaS = new BallSensor(out -> {
            fill(out, bw.copaMm < 10 ? bw.copa : BallType.NONE, bw.copa.isNectar() ? 4.5 : 16.2);
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
        // La de arriba, quieta, no toca la de la copa (CAD: z 144.8 con NECTAR sobre NECTAR)
        if (bw.upper.isBall() && bw.copa.isBall() && !bw.lifting.isBall()) {
            double gap = bw.upperZOrDefault() - copaCenterZ(bw.copa, bw.copaMm) - radius(bw.upper) - radius(bw.copa);
            bw.minTopGapMm = Math.min(bw.minTopGapMm, gap);
            if (gap < -0.01) {
                bw.hazard = true;
                bw.hazardWhy = String.format("la de arriba (centro en z %.1f) toca la de la copa", bw.upperZOrDefault());
            }
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

        // Elevador (su encoder cuenta la superficie de las Gecko)
        if (!bw.encoderDead) elev.pos += ePow * LIFT_MM_S_PER_POWER / FeederCfg.ELEVATOR_MM_PER_TICK * dt;
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
                bw.upperZ = Double.NaN;
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
        // Subir a la etapa de arriba: con la copa arriba las Gecko de z 95 toman la pelota y la
        // suben a 1920 mm/s por unidad de potencia; si el elevador para, se queda donde va.
        if (!bw.lifting.isBall() && ePow > 0.05 && ePow <= 0.5 && bw.copaMm >= 50 && bw.copa.isBall()
                && !bw.upper.isBall() && !bw.liftBlocked) {
            if (bw.liftLoses) {                   // sale de la copa y se atora en el tubo, abajo del V3
                bw.tube = bw.copa;
            } else {
                bw.lifting = bw.copa;
                bw.liftZ = copaCenterZ(bw.copa, bw.copaMm);
            }
            bw.copa = BallType.NONE;
        }
        if (bw.lifting.isBall()) {
            if (ePow > 0.05) {
                bw.liftZ += bw.ballSpeedRatio * LIFT_MM_S_PER_POWER * ePow * dt;   // el encoder cuenta la superficie
                if (bw.liftZ >= flywheelContactZ(bw.lifting)) {
                    bw.hazard = true;
                    bw.hazardWhy = String.format("tocó el flywheel subiendo a su etapa (centro en z %.1f)", bw.liftZ);
                }
            } else {                              // el elevador paró: ahí se queda
                bw.upper = bw.lifting;
                bw.upperZ = bw.liftZ;
                bw.lifting = BallType.NONE;
            }
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

    /** Caídas de cálculo (documento del CAD v138, 4.8): 60-100 rpm con POLLEN, 120-190 con NECTAR. */
    static int calcDipsConfirmed(double[] worstMeasureErr) {
        Fakes.Motor m = new Fakes.Motor();
        Flywheel fw = new Flywheel(m.proxy());
        fw.setEnabled(true);
        double tpr = FlywheelCfg.TICKS_PER_REV, target = 3000;
        fw.setTargetRpm(target);
        long now = 0;
        for (int i = 0; i < 50; i++) { now += 10 * MS; m.vel = target / 60 * tpr; fw.update(now); }
        fw.consumeShotDip();
        int confirmed = 0;
        for (double drop : new double[]{60, 100, 120, 190}) {
            fw.resetMaxDrop();
            for (int i = 0; i < 40; i++) {
                now += 10 * MS;
                m.vel = (i < 5 ? target - drop : target) / 60 * tpr;
                fw.update(now);
                if (fw.consumeShotDip()) confirmed++;
            }
            worstMeasureErr[0] = Math.max(worstMeasureErr[0], Math.abs(fw.getMaxDropRpm() - drop));
        }
        return confirmed;
    }

    static void testFlywheelCalcDips() {
        double[] err = {0};
        int confirmed = calcDipsConfirmed(err);
        check("flywheel: \"Tune: Tiro\" ve la caída de cada tiro", err[0] < 1, String.format("error %.1f rpm", err[0]));
        System.out.printf("      (riesgo hasta medir: con SHOT_DIP_RPM %.0f, %d de 4 caídas de cálculo (60-190 rpm) confirman el tiro)%n",
                FlywheelCfg.SHOT_DIP_RPM, confirmed);
        double saved = FlywheelCfg.SHOT_DIP_RPM;
        FlywheelCfg.SHOT_DIP_RPM = 50;
        int low = calcDipsConfirmed(new double[]{0});
        FlywheelCfg.SHOT_DIP_RPM = saved;
        check("flywheel: con el umbral abajo de la caída más chica, todas confirman", low == 4, low + " de 4");
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
                upperReading(bw, out);
                return true;
            }, SensorCfg.UPPER, 0.04);
            BallSensor copaS = new BallSensor(out -> {
                fill(out, bw.copaMm < 10 ? bw.copa : BallType.NONE, bw.copa.isNectar() ? 4.5 : 16.2);
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

    /** Un tiro de verdad con la caída de cálculo de una NECTAR (150 rpm) pasando por el Flywheel. */
    static void testFeederCalcDipUnconfirmed() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.NECTAR_RED;
        Fakes.Motor fm = new Fakes.Motor();
        Flywheel fw = new Flywheel(fm.proxy());
        fw.setEnabled(true);
        fw.setTargetRpm(3500);
        double tpr = FlywheelCfg.TICKS_PER_REV;
        int dipLeft = 0, shots = 0;
        for (int i = 0; i < 50; i++) { r.now += 10 * MS; fm.vel = 3500 / 60.0 * tpr; fw.update(r.now); }
        r.f.requestShot();
        for (int i = 0; i < 200; i++) {
            r.now += 10 * MS;
            fm.vel = (dipLeft > 0 ? 3500 - 150 : 3500) / 60.0 * tpr;
            if (dipLeft > 0) dipLeft--;
            fw.update(r.now);
            r.f.update(r.now, false, false, false, fw.consumeShotDip(), true);
            shots += r.f.consumeShots();
            physics(r.bw, r.fila, r.copa, r.elev, 0.01, r.order);
            if (r.bw.dip) {
                r.bw.dip = false;
                dipLeft = 5;
            }
        }
        check(String.format("con SHOT_DIP_RPM %.0f una caída de 150 rpm no confirma: \"sin confirmar\" (medirlo)",
                FlywheelCfg.SHOT_DIP_RPM), FlywheelCfg.SHOT_DIP_RPM < 150 ? shots == 1
                : shots == 0 && r.f.isClearPending() && r.bw.launched == 1, shots + " contados, salió " + r.bw.launched);
    }

    /** Cuenta las subidas de la copa y las que se abortaron sin llegar a LIFT. */
    static int[] runRig(FeederRig r, int steps, boolean intakeFeeding) {
        int rises = 0, aborts = 0;
        Feeder.State prev = r.f.getState();
        for (int i = 0; i < steps; i++) {
            r.step(intakeFeeding, true);
            Feeder.State s = r.f.getState();
            if (s == Feeder.State.COPA_UP && prev != Feeder.State.COPA_UP) rises++;
            if (s == Feeder.State.COPA_DOWN && prev == Feeder.State.COPA_UP) aborts++;
            prev = s;
        }
        return new int[]{rises, aborts};
    }

    /*
     * Fin de LIFT (documento del CAD v138, 4.7): la de la copa sube a su etapa y la siguiente
     * entra a la copa. La de arriba tiene que quedar sin tocar la de la copa (z 144.8 con NECTAR
     * sobre NECTAR) y sin llegar al flywheel (NECTAR 156.5, POLLEN 170.8).
     */
    static void testFeederLiftEnd() {
        BallType N = BallType.NECTAR_RED, P = BallType.POLLEN;
        for (double cone : new double[]{30, 45}) {
            for (BallType[] pair : new BallType[][]{{N, N}, {P, P}, {P, N}, {N, P}}) {
                FeederRig r = new FeederRig();
                r.bw.coneDeg = cone;
                r.bw.copa = pair[0];
                r.bw.fila = pair[1];
                int[] ra = runRig(r, 400, false);
                double z = r.bw.upperZOrDefault();
                double low = copaCenterZ(pair[1], 0) + radius(pair[0]) + radius(pair[1]);
                boolean staged = r.bw.upper == pair[0] && r.bw.copa == pair[1];
                check(String.format("fin de LIFT, cono %.0f°, %s sobre %s: sin tocar la de la copa y sostenida por las Gecko",
                                cone, pair[0] == N ? "NECTAR" : "POLLEN", pair[1] == N ? "NECTAR" : "POLLEN"),
                        staged && ra[0] == 1 && !r.bw.hazard && z >= low && z <= geckoHoldZ(pair[0])
                                && r.f.liftTrace().contains(FeederCfg.LIFT_BY_ENCODER ? "con el encoder" : "pasó a"),
                        String.format("centro en z %.1f (%.1f a %.1f); %s %s", z, low, geckoHoldZ(pair[0]),
                                r.f.liftTrace(), r.bw.hazardWhy));
                java.util.regex.Matcher mz = java.util.regex.Pattern.compile("z ≈ ([0-9]+[.][0-9]+)").matcher(r.f.liftTrace());
                if (cone == 30 && pair[1] == N) {
                    check(String.format("el encoder del elevador da dónde quedó la %s (±1 mm)", pair[0] == N ? "NECTAR" : "POLLEN"),
                            mz.find() && Math.abs(Double.parseDouble(mz.group(1)) - z) <= 1.0, r.f.liftTrace());
                }
            }
        }
        // Referencia: si el V3 de arriba solo viera ±20°, la de arriba ya en su etapa (~22°) no se vería
        FeederRig narrow = new FeederRig();
        narrow.bw.coneDeg = 20;
        narrow.bw.copa = N;
        narrow.bw.fila = N;
        runRig(narrow, 600, false);
        System.out.printf("      (referencia, cono de 20°: %s; la de arriba ya en su etapa tiene que verse: MEDIR)%n",
                narrow.bw.hazard ? narrow.bw.hazardWhy : "sin choque");
    }

    /** La de la copa sube a su etapa con el V3 de arriba ruidoso o con un parpadeo al empezar. */
    static FeederRig liftRig(double noiseMm, long seed, double floorMm, double dropoutS) {
        FeederRig r = new FeederRig();
        r.bw.copa = BallType.NECTAR_RED;
        r.bw.fila = BallType.NECTAR_RED;
        r.bw.upperNoiseMm = noiseMm;
        r.bw.upperFloorMm = floorMm;
        r.bw.noise = new Random(seed);
        boolean dropoutSet = false;
        for (int i = 0; i < 400; i++) {
            r.step(false, true);
            if (!dropoutSet && dropoutS > 0 && r.f.getState() == Feeder.State.LIFT) {
                r.bw.flickerFromS = r.bw.t;           // deja de verla justo al empezar a subir
                r.bw.flickerToS = r.bw.t + dropoutS;
                dropoutSet = true;
            }
        }
        return r;
    }

    /** NECTAR sobre NECTAR: sin tocar la de la copa (z 144.8) y sostenida por las Gecko (z 147.0). */
    static boolean stagedInWindow(FeederRig r) {
        double z = r.bw.upperZOrDefault();
        return r.bw.upper == BallType.NECTAR_RED && !r.bw.hazard
                && z >= copaCenterZ(BallType.NECTAR_RED, 0) + 2 * radius(BallType.NECTAR_RED)
                && z <= geckoHoldZ(BallType.NECTAR_RED);
    }

    /**
     * NECTAR sobre NECTAR con ruido en el V3 de arriba, 30 intentos: {cuántas quedan arriba de lo
     * que sostienen las Gecko (o tocan el flywheel), cuántas quedan tocando la de la copa, z más baja}.
     */
    static double[] liftNoiseRuns(double sigma) {
        int high = 0, low = 0;
        double lowest = 999;
        for (long seed = 1; seed <= 30; seed++) {
            FeederRig r = liftRig(sigma, seed, 0, 0);
            double z = r.bw.upperZOrDefault();
            if (z > geckoHoldZ(BallType.NECTAR_RED) || r.bw.hazardWhy.contains("flywheel")) high++;
            if (z < copaCenterZ(BallType.NECTAR_RED, 0) + 2 * radius(BallType.NECTAR_RED)) low++;
            lowest = Math.min(lowest, z);
        }
        return new double[]{high, low, lowest};
    }

    static void testFeederLiftNoise() {
        // Con el encoder el ruido del V3 no cambia dónde queda (el V3 solo confirma que la vio)
        for (double sigma : new double[]{0.3, 0.5}) {
            double[] n = liftNoiseRuns(sigma);
            check(String.format("fin de LIFT con encoder y ruido de %.1f mm en el V3: entre la de la copa y las Gecko (30)",
                    sigma), n[0] == 0 && n[1] == 0, String.format("%d arriba, %d tocando la de la copa", (int) n[0], (int) n[1]));
        }
        FeederRig d = liftRig(0, 1, 0, 0.03);
        check("fin de LIFT: un parpadeo de 30 ms al empezar a subir no la deja baja", stagedInWindow(d),
                String.format("z %.1f; %s", d.bw.upperZOrDefault(), d.f.liftTrace()));

        // La pelota más lenta que las Gecko (CAD v146: entre la mitad y el total de la superficie)
        double savedN = FeederCfg.LIFT_BALL_MM_PER_TICK_NECTAR;
        try {
            FeederRig slow = new FeederRig();
            slow.bw.ballSpeedRatio = 0.8;
            slow.bw.copa = BallType.NECTAR_RED;
            slow.bw.fila = BallType.NECTAR_RED;
            runRig(slow, 400, false);
            double zs = slow.bw.upperZOrDefault();
            check("pelota 20 % más lenta que las Gecko, sin calibrar: queda más baja, nunca más alta",
                    zs <= geckoHoldZ(BallType.NECTAR_RED) && !slow.bw.hazardWhy.contains("flywheel"),
                    String.format("z %.1f (toca la de la copa debajo de 144.8: calibrar LIFT_BALL_MM_PER_TICK)", zs));
            FeederCfg.LIFT_BALL_MM_PER_TICK_NECTAR = 0.8 * FeederCfg.ELEVATOR_MM_PER_TICK;   // ya medido
            FeederRig cal = new FeederRig();
            cal.bw.ballSpeedRatio = 0.8;
            cal.bw.copa = BallType.NECTAR_RED;
            cal.bw.fila = BallType.NECTAR_RED;
            runRig(cal, 400, false);
            check("pelota 20 % más lenta, ya calibrada: vuelve a quedar entre la de la copa y las Gecko",
                    stagedInWindow(cal), String.format("z %.1f; %s", cal.bw.upperZOrDefault(), cal.f.liftTrace()));
        } finally {
            FeederCfg.LIFT_BALL_MM_PER_TICK_NECTAR = savedN;
        }

        // Sin encoder (sin cable): avisa y termina con el V3
        FeederRig dead = new FeederRig();
        dead.bw.encoderDead = true;
        dead.bw.copa = BallType.NECTAR_RED;
        dead.bw.fila = BallType.NECTAR_RED;
        runRig(dead, 400, false);
        check("sin encoder del elevador: avisa y termina la subida con el V3 de arriba",
                stagedInWindow(dead) && dead.f.getWarning().contains("encoder") && dead.f.liftTrace().contains("con el V3"),
                String.format("z %.1f; aviso '%s'", dead.bw.upperZOrDefault(), dead.f.getWarning()));

        // Con el V3 (sin encoder): el ruido reparte el final ±3 mm alrededor de lo que sostienen las Gecko
        boolean savedEnc = FeederCfg.LIFT_BY_ENCODER;
        FeederCfg.LIFT_BY_ENCODER = false;
        try {
            for (double sigma : new double[]{0.3, 0.5}) {
                double[] n = liftNoiseRuns(sigma);
                System.out.printf("      (referencia, con el V3 y ruido de %.1f mm: %d de 30 arriba de lo que sostienen las Gecko,"
                        + " %d tocando la de la copa, la más baja en z %.1f)%n", sigma, (int) n[0], (int) n[1], n[2]);
            }
            FeederRig flat = liftRig(0, 1, 11, 0);   // la distancia nunca baja de 11 mm (8.5 en el lente)
            check("con el V3 y una curva más plana: el tope de recorrido no la deja tocar el flywheel",
                    !flat.bw.hazardWhy.contains("flywheel") && flat.bw.upper == BallType.NECTAR_RED,
                    String.format("z %.1f; %s", flat.bw.upperZOrDefault(), flat.f.liftTrace()));
        } finally {
            FeederCfg.LIFT_BY_ENCODER = savedEnc;
        }
    }

    static void testFeederTopFlicker() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.NECTAR_RED;         // la de arriba ya en su etapa (z 150)
        r.bw.copa = BallType.NECTAR_RED;
        for (int i = 0; i < 50; i++) r.step(false, true);
        r.bw.flickerFromS = r.bw.t;               // el V3 de arriba no la ve 100 ms
        r.bw.flickerToS = r.bw.t + 0.10;
        for (int i = 0; i < 100; i++) r.step(false, true);
        check("un parpadeo de 100 ms del V3 de arriba no sube la copa contra la de arriba",
                r.bw.maxCopaMm < 1 && !r.bw.hazard, String.format("copa hasta %.1f mm %s", r.bw.maxCopaMm, r.bw.hazardWhy));
    }

    static void testFeederCopaUpOwnBall() {
        FeederRig r = new FeederRig();
        r.bw.coneDeg = 45;                        // ve la NECTAR de la copa desde ~21 mm de subida
        r.bw.copa = BallType.NECTAR_RED;
        int[] ra = runRig(r, 300, false);
        check("al subir la copa, ver su propia pelota no aborta (COPA_UPPER_BLIND_MM)",
                ra[0] == 1 && ra[1] == 0 && r.bw.upper == BallType.NECTAR_RED && !r.bw.hazard,
                ra[0] + " subidas, " + ra[1] + " abortadas, arriba " + r.bw.upper);
    }

    static void testFeederNectarStuckInCopa() {
        FeederRig r = new FeederRig();
        r.bw.copa = BallType.NECTAR_RED;          // con la copa arriba el V3 de arriba la ve a 12.5 mm...
        r.bw.liftBlocked = true;                  // ...pero nunca sale de la copa
        int[] ra = runRig(r, 1500, false);
        check("una NECTAR que se ve arriba pero nunca sale de la copa no se reintenta para siempre",
                ra[0] == FeederCfg.REFILL_MAX_FAILS && r.f.isRefillBlocked() && !r.bw.hazard,
                ra[0] + " subidas " + r.bw.hazardWhy);
    }

    static void testFeederSeatThird() {
        FeederRig r = new FeederRig();
        r.bw.upper = BallType.POLLEN;             // arriba ocupada: la copa no sube
        r.bw.copa = BallType.POLLEN;
        r.bw.throat = BallType.NECTAR_RED;        // la 3a llega por la fila
        double seatS = 0;
        for (int i = 0; i < 200; i++) {
            boolean filaBefore = r.bw.fila.isBall();
            r.step(true, true);
            if (filaBefore && r.fila.effective() * Math.signum(FeederCfg.FILA_POWER_IN) > 0.5) seatS += 0.01;
        }
        check("la 3a se asienta: la fila sigue FILA_SEAT_S después del primer corte de BB3",
                r.bw.fila == BallType.NECTAR_RED && Math.abs(seatS - FeederCfg.FILA_SEAT_S) <= 0.015 && !r.fila.pwmOn,
                String.format("%.2f s más", seatS));
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
        for (int i = 0; i < 150; i++) { now += 10 * MS; in.update(now, false); }
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

    /**
     * Las pruebas de la lengua son de un robot ya calibrado (CALIBRATED = true) con los µs de
     * cálculo de Config: 1002 / 1516 / 1998 (documento del CAD v138).
     */
    static boolean calibrate() {
        boolean saved = IntakeLenguaCfg.CALIBRATED;
        IntakeLenguaCfg.CALIBRATED = true;
        return saved;
    }

    static void restoreCalibrated(boolean saved) {
        IntakeLenguaCfg.CALIBRATED = saved;
    }

    static IntakeLengua lengua(Fakes.Motor m, Fakes.PosServo sv, boolean[] throatBeam, BallType[] seen) {
        return lengua(m, sv, new boolean[]{false}, throatBeam, seen);
    }

    static IntakeLengua lengua(Fakes.Motor m, Fakes.PosServo sv, boolean[] mouthBeam, boolean[] throatBeam, BallType[] seen) {
        return new IntakeLengua(m.proxy(), sv.proxy(), new BeamSensor(Fakes.digital(() -> mouthBeam[0]), true),
                new BeamSensor(Fakes.digital(() -> throatBeam[0]), true),
                new BallSensor(out -> { fill(out, seen[0], 9); return true; }, SensorCfg.THROAT, 0.0),
                null, Alliance.RED);
    }

    static void testLenguaInitBallInThroat() {
        boolean saved = calibrate();
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
                    && near(sv.position, TongueServo.positionFor(IntakeLenguaCfg.FOLDED_US)), "");
        } finally {
            restoreCalibrated(saved);
        }
    }
    static void testLenguaInitBallAtMouth() {
        boolean saved = calibrate();
        try {
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            boolean[] mouth = {true};                 // la 4a bajo los dedos: corta BB1, no la garganta
            IntakeLengua in = lengua(m, sv, mouth, new boolean[]{false}, new BallType[]{BallType.NONE});
            long now = 0;
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            check("lengua: con pelota en la boca (BB1) y la garganta libre no se pliega en INIT y avisa",
                    !sv.everCommanded && !in.isStartReady() && in.warning().contains("boca"), "aviso '" + in.warning() + "'");
            mouth[0] = false;
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            check("lengua: ya libre la boca, se pliega", in.isStartReady() && !sv.pwmOn, "");
        } finally {
            restoreCalibrated(saved);
        }
    }

    /** Tiempo que el modelo tarda en desplegar desde plegada: al empuje, rampa y asentarse. */
    static double deployTimeS() {
        double servoUsS = IntakeLenguaCfg.SERVO_DEG_S / TongueServo.DEG_PER_US;
        double rampUsS = IntakeLenguaCfg.DEPLOY_RAMP_DEG_S / TongueServo.DEG_PER_US;
        return Math.abs(IntakeLenguaCfg.FOLDED_US - IntakeLenguaCfg.PUSH_US) / servoUsS
                + Math.abs(IntakeLenguaCfg.PUSH_US - IntakeLenguaCfg.DOWN_US) / rampUsS + IntakeLenguaCfg.SETTLE_S;
    }

    static void testLenguaTongue() {
        boolean saved = calibrate();
        try {
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            boolean[] beam = {false};
            IntakeLengua in = lengua(m, sv, beam, new BallType[]{BallType.NONE});
            double servoUsS = IntakeLenguaCfg.SERVO_DEG_S / TongueServo.DEG_PER_US;
            double foldS = Math.abs(IntakeLenguaCfg.DOWN_US - IntakeLenguaCfg.FOLDED_US) / servoUsS + IntakeLenguaCfg.SETTLE_S;
            long now = 0;
            double readyAt = -1;
            for (int i = 1; i <= 150; i++) {
                now += 10 * MS;
                if (in.initStep(now) && readyAt < 0) readyAt = i * 0.01;
            }
            check("lengua: en INIT va directo a plegada y luego se suelta (PWM apagado)", in.isStartReady() && !sv.pwmOn
                    && near(sv.position, TongueServo.positionFor(IntakeLenguaCfg.FOLDED_US)) && m.power == 0
                    && Math.abs(readyAt - foldS) <= 0.03, String.format("lista a %.2f s (modelo %.2f s)", readyAt, foldS));

            in.setCommand(Intake.Command.IN);         // Play con 4 pelotas e intake prendido
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.update(now, true); }
            check("lengua: al dar Play con 4 pelotas se queda plegada y suelta",
                    in.getTongue() == TongueServo.Position.FOLDED && !sv.pwmOn && m.power == 0, "" + in.getTongue());

            // Ya hay lugar: despliega (salto al empuje, rampa hasta abajo) y solo entonces jala
            double pullAt = -1, prevPos = sv.position, worstRampUsS = 0;
            boolean firstIsPush = false, sawPush = false, pulledEarly = false;
            for (int i = 1; i <= 200; i++) {
                now += 10 * MS;
                in.update(now, false);
                if (i == 1) firstIsPush = near(sv.position, TongueServo.positionFor(IntakeLenguaCfg.PUSH_US));
                double us = TongueServo.MIN_US + sv.position * (TongueServo.MAX_US - TongueServo.MIN_US);
                if (sawPush) worstRampUsS = Math.max(worstRampUsS, Math.abs(sv.position - prevPos) * 2000 / 0.01);
                if (Math.abs(us - IntakeLenguaCfg.PUSH_US) < 0.5 || (sawPush && us != IntakeLenguaCfg.PUSH_US)) sawPush = true;
                prevPos = sv.position;
                if (m.power > 0 && pullAt < 0) pullAt = i * 0.01;
                if (m.power > 0 && !in.isTongueSettled()) pulledEarly = true;
            }
            double deployS = deployTimeS();
            check("lengua: al desplegar, primero salta al empuje", firstIsPush, "");
            check("lengua: de empuje a abajo, rampa de 120°/s como máximo (800 µs/s)", worstRampUsS <= 800 + 1,
                    String.format("peor %.0f µs/s", worstRampUsS));
            check("lengua: M_F jala hasta que la lengua llegó abajo", !pulledEarly && in.getTongue() == TongueServo.Position.DOWN
                    && sv.pwmOn && near(sv.position, TongueServo.positionFor(IntakeLenguaCfg.DOWN_US))
                    && Math.abs(pullAt - deployS) <= 0.03, String.format("jala a %.2f s (modelo %.2f s)", pullAt, deployS));
            check("lengua: jalando, la fila puede traer la siguiente", in.isFeeding(), "");

            beam[0] = true;                           // llega la 4a y se queda en la garganta
            boolean moved = false;
            for (int i = 0; i < 200; i++) {
                now += 10 * MS;
                in.update(now, true);
                if (in.getTongue() != TongueServo.Position.DOWN
                        || !near(sv.position, TongueServo.positionFor(IntakeLenguaCfg.DOWN_US))) moved = true;
            }
            check("lengua: con 4 pelotas se queda ABAJO y M_F para en BRAKE (no empuja la 4a)", !moved && sv.pwmOn
                    && m.power == 0 && m.zpb == DcMotor.ZeroPowerBehavior.BRAKE, "" + in.getTongue());

            beam[0] = false;                          // salió un tiro
            in.setCommand(Intake.Command.STOP);
            for (int i = 0; i < 50; i++) { now += 10 * MS; in.update(now, false); }
            check("lengua: con lugar pero intake apagado se queda abajo y quieta",
                    in.getTongue() == TongueServo.Position.DOWN && m.power == 0, "" + in.getTongue());
            in.setCommand(Intake.Command.IN);
            for (int i = 0; i < 20; i++) { now += 10 * MS; in.update(now, false); }
            check("lengua: al prender el intake jala de inmediato (ya está abajo)", m.power > 0, "");
        } finally {
            restoreCalibrated(saved);
        }
    }

    static void testLenguaRejectFromFolded() {
        boolean saved = calibrate();
        try {
            BallType[] seen = {BallType.NONE};
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            IntakeLengua in = lengua(m, sv, new boolean[]{false}, seen);
            long now = 0;
            for (int i = 0; i < 150; i++) { now += 10 * MS; in.initStep(now); }
            in.setCommand(Intake.Command.IN);
            for (int i = 0; i < 20; i++) { now += 10 * MS; in.update(now, true); }   // lleno: sigue plegada
            seen[0] = BallType.NECTAR_BLUE;           // una rival en la garganta con la lengua plegada
            boolean badWhileMoving = false;
            double reverseAt = -1;
            for (int i = 1; i <= 200; i++) {
                now += 10 * MS;
                in.update(now, true);
                boolean down = in.getTongue() == TongueServo.Position.DOWN && in.isTongueSettled();
                if (!down && (m.power != 0 || in.wantsFilaReverse() || !in.holdsFila())) badWhileMoving = true;
                if (m.power < 0 && in.wantsFilaReverse() && reverseAt < 0) reverseAt = i * 0.01;
            }
            double deployS = deployTimeS();
            check("lengua: rival con la lengua plegada -> primero baja, con M_F y la fila quietos", !badWhileMoving, "");
            check("lengua: ya abajo, reversa de M_F y de la fila", Math.abs(reverseAt - deployS) <= 0.03,
                    String.format("reversa a %.2f s (modelo %.2f s)", reverseAt, deployS));
        } finally {
            restoreCalibrated(saved);
        }
    }

    /** El intake con la cuenta de G407 como la lleva Robot: lleno por la cuenta o por 3 etapas + entrada. */
    static final class EntryRig {
        final Fakes.Motor m = new Fakes.Motor();
        final Fakes.PosServo sv = new Fakes.PosServo();
        final boolean[] mouth = {false}, throat = {false};
        final IntakeLengua in = lengua(m, sv, mouth, throat, new BallType[]{BallType.NONE});
        final BallInventory inv = new BallInventory();
        int stages = 3;                           // arriba, copa y fila ocupadas
        boolean accepting = false;                // con la fila ocupada, la fila no jala
        boolean filaBall = true;                  // BB3
        long now = 0;
        boolean full;

        void step() {
            now += 10 * MS;
            full = inv.isFull(stages, in.hasBallAtEntry());
            in.update(now, full, accepting, filaBall);
            for (int k = in.consumeEntered(); k > 0; k--) inv.entered();
            for (int k = in.consumeRejected(); k > 0; k--) inv.left();
            inv.reconcile(stages);
        }
    }

    /*
     * Documento del CAD v138, 4.5: con la lengua, la 4a se queda en la boca sin cortar el haz de
     * la garganta. Tiene que entrar a LLENO por BB1, sin reversas de "atasco", y contarse una
     * sola vez cuando por fin cruza la garganta.
     */
    static void testLenguaFourthAtMouth() {
        EntryRig r = new EntryRig();
        r.inv.set(3);
        r.in.setCommand(Intake.Command.IN);
        for (int i = 0; i < 150; i++) r.step();   // jalando, con la lengua abajo
        r.mouth[0] = true;                        // llega la 4a y se queda en la boca
        int reverses = 0;
        boolean wasRev = false;
        double fullAt = -1;
        for (int i = 1; i <= 100; i++) {
            r.step();
            boolean rev = r.m.power < 0;
            if (rev && !wasRev) reverses++;
            wasRev = rev;
            if (r.full && fullAt < 0) fullAt = i * 0.01;
        }
        check("4a en la boca sin cortar la garganta: LLENO, M_F en BRAKE y lengua abajo, sin reversas",
                fullAt > 0 && fullAt <= IntakeLenguaCfg.FULL_MOUTH_S + 0.03 && r.m.power == 0
                        && r.m.zpb == DcMotor.ZeroPowerBehavior.BRAKE && r.in.getTongue() == TongueServo.Position.DOWN
                        && reverses == 0 && r.inv.count() == 3,
                String.format("lleno a %.2f s, %d reversas, cuenta %d", fullAt, reverses, r.inv.count()));

        EntryRig k = new EntryRig();              // la fila no sabe del intake: FULL_MOUTH_S < UNJAM_MOUTH_S
        k.inv.set(3);
        k.accepting = true;
        k.in.setCommand(Intake.Command.IN);
        for (int i = 0; i < 150; i++) k.step();
        k.mouth[0] = true;
        int kRev = 0;
        for (int i = 0; i < 100; i++) {
            k.step();
            if (k.m.power < 0) kRev++;
        }
        check("4a en la boca aunque la fila parezca jalar: LLENO antes de la regla de atasco",
                k.full && k.m.power == 0 && kRev == 0, kRev + " ciclos en reversa");

        EntryRig s = new EntryRig();              // la 3a NECTAR sin asentar corta la garganta
        s.inv.set(2);
        s.stages = 2;
        s.filaBall = false;
        s.accepting = true;
        s.in.setCommand(Intake.Command.IN);
        for (int i = 0; i < 150; i++) s.step();
        s.throat[0] = true;                       // la 3a cruza la garganta y se cuenta...
        for (int i = 0; i < 10; i++) s.step();
        s.stages = 3;                             // ...llega a BB3 y la fila se para: sigue en el haz
        s.filaBall = true;
        s.accepting = false;
        boolean stopped = false;
        for (int i = 0; i < 150; i++) {
            s.step();
            if (i > 120 && (s.full || s.m.power <= 0)) stopped = true;
        }
        check("la 3a sin asentar cortando la garganta no es la 4a: sigue jalando", !stopped && s.inv.count() == 3,
                "lleno " + s.full + ", M_F " + s.m.power);

        boolean leftFull = false;                 // variante: corta la garganta a ratos (NECTAR a ~1-2 mm)
        for (int i = 0; i < 200; i++) {
            r.throat[0] = (i % 13) < 4;
            r.step();
            if (!r.full || r.m.power != 0) leftFull = true;
        }
        r.throat[0] = false;
        r.step();
        check("4a cortando la garganta a ratos: no se cuenta dos veces ni sale de LLENO",
                !leftFull && r.inv.count() == 3, "cuenta " + r.inv.count());

        r.stages = 2;                             // sale un tiro: hay lugar, pero la fila sigue ocupada
        r.inv.left();                             // mientras se rellena la copa
        reverses = 0;
        wasRev = false;
        boolean pushed = false;
        for (int i = 0; i < 150; i++) {
            r.throat[0] = (i % 13) < 4;           // y corta la garganta a ratos mientras espera
            r.step();
            boolean rev = r.m.power < 0;
            if (rev && !wasRev) reverses++;
            wasRev = rev;
            if (r.m.power > 0) pushed = true;
        }
        r.throat[0] = false;
        r.step();
        check("con lugar pero la fila sin jalar, la 4a espera: M_F en BRAKE, sin reversas ni cuentas de más",
                !pushed && reverses == 0 && !r.in.warning().startsWith("atasco") && r.inv.count() == 2,
                reverses + " reversas, cuenta " + r.inv.count() + ", aviso '" + r.in.warning() + "'");
        r.accepting = true;                       // la fila carga la copa y jala la siguiente
        r.step();
        r.step();
        boolean pulls = r.m.power > 0;
        r.throat[0] = true;                       // la 4a cruza la garganta
        for (int i = 0; i < 10; i++) r.step();
        r.throat[0] = false;
        r.mouth[0] = false;
        for (int i = 0; i < 20; i++) r.step();
        check("ya que la fila jala, la 4a entra y se cuenta una sola vez", pulls && r.inv.count() == 3,
                "cuenta " + r.inv.count());
    }

    static void testLenguaOverCurrent() {
        BallType[] seen = {BallType.NONE};
        Fakes.Motor m = new Fakes.Motor();
        Fakes.PosServo sv = new Fakes.PosServo();
        IntakeLengua in = lengua(m, sv, new boolean[]{false}, seen);
        check("M_F: el límite de corriente del hub se pone en MF_CURRENT_LIMIT_A",
                m.currentAlertA == IntakeLenguaCfg.MF_CURRENT_LIMIT_A, m.currentAlertA + " A");
        long now = warmUp(in, 0);
        double posDown = sv.position;
        m.overCurrent = true;                     // atasco: M_F arriba de MF_CURRENT_LIMIT_A
        double stopAt = -1;
        for (int i = 1; i <= 60; i++) {
            now += 10 * MS;
            in.update(now, false);
            if (m.power == 0 && stopAt < 0) stopAt = i * 0.01;
        }
        check("M_F con sobrecorriente: se detiene en BRAKE tras MF_OVERCURRENT_S, avisa y la lengua no se mueve",
                Math.abs(stopAt - IntakeLenguaCfg.MF_OVERCURRENT_S) <= 0.02 && in.warning().contains("sobrecorriente")
                        && near(sv.position, posDown) && m.zpb == DcMotor.ZeroPowerBehavior.BRAKE,
                String.format("se detuvo a %.2f s, aviso '%s'", stopAt, in.warning()));
        m.overCurrent = false;
        for (int i = 0; i < 30; i++) { now += 10 * MS; in.update(now, false); }
        boolean stays = m.power == 0;
        in.setCommand(Intake.Command.STOP);
        for (int i = 0; i < 5; i++) { now += 10 * MS; in.update(now, false); }
        in.setCommand(Intake.Command.IN);
        for (int i = 0; i < 5; i++) { now += 10 * MS; in.update(now, false); }
        check("M_F con sobrecorriente: vuelve a jalar solo cuando el piloto vuelve a mandar el intake",
                stays && m.power > 0, "");

        seen[0] = BallType.NECTAR_BLUE;           // sacando una rival, M_F se atora
        for (int i = 0; i < 10; i++) { now += 10 * MS; in.update(now, false); }
        m.overCurrent = true;
        for (int i = 0; i < 40; i++) { now += 10 * MS; in.update(now, false); }
        check("M_F con sobrecorriente sacando una rival: se detiene y la fila se queda quieta",
                m.power == 0 && in.holdsFila() && !in.wantsFilaReverse(), "aviso '" + in.warning() + "'");
    }

    static void testLenguaUncalibrated() {
        boolean saved = IntakeLenguaCfg.CALIBRATED;
        IntakeLenguaCfg.CALIBRATED = false;
        try {
            Fakes.Motor m = new Fakes.Motor();
            Fakes.PosServo sv = new Fakes.PosServo();
            IntakeLengua in = lengua(m, sv, new boolean[]{false}, new BallType[]{BallType.NONE});
            long now = 0;
            for (int i = 0; i < 100; i++) { now += 10 * MS; in.initStep(now); }
            boolean initOk = !sv.everCommanded && !in.isStartReady() && in.warning().contains("sin calibrar");
            boolean moved = false;
            for (Intake.Command c : new Intake.Command[]{Intake.Command.IN, Intake.Command.OUT}) {
                in.setCommand(c);
                for (int i = 0; i < 150; i++) {
                    now += 10 * MS;
                    in.update(now, false);
                    if (m.power != 0 || sv.everCommanded) moved = true;
                }
            }
            check("lengua sin calibrar (CALIBRATED = false): ni la lengua ni M_F se mueven, y avisa",
                    initOk && !moved, "aviso '" + in.warning() + "'");
        } finally {
            IntakeLenguaCfg.CALIBRATED = saved;
        }
    }
}
