package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Config.Names;
import org.firstinspires.ftc.teamcode.Config.VisionCfg;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Lee el Limelight y entrega la abertura de la CELL de ARRIBA de nuestra alianza, vista
 * desde el eje de la torreta.
 *
 *  1. Tiempos: getControlHubTimeStampNanos() del SDK es hora de reloj, no la base de
 *     System.nanoTime(); el instante de la foto sale de getStaleness() y las latencias.
 *  2. Fotos repetidas: el SDK crea un resultado cada 10 ms aunque la cámara no haya
 *     tomado otra foto. La foto se reconoce por getTimestamp(), el reloj del Limelight.
 *  3. Arriba o abajo: desde donde se tira se ven las dos CELLS de la alianza. Con 2 o más
 *     tags de una fila se MIDE su altura por la separación entre tags (HiveGeometry); con
 *     Full 3D, también por la pose. La de arriba está a ~49.9 in y la de abajo a ~35.6 in.
 *     HiveBelief guarda la decisión; si una foto la contradice, esa foto no se usa.
 *  4. Se apunta al centro de la ABERTURA, 3.41 in afuera de la fila de tags.
 *
 * No usa el área "ta" de los tags: su escala en el Limelight es dudosa (0-1 o 0-100).
 * pipelineSwitch() solo se llama en INIT: es un POST que puede bloquear hasta 15 s.
 */
public class CellVision {
    private final Limelight3A limelight;    // null en la simulación
    private final Alliance alliance;
    private final ClusterSolver solver = new ClusterSolver(VisionCfg.SLOPE_MEMORY_S);
    private final HiveBelief belief;

    private double lastLlTimestamp = Double.NaN;

    // Para telemetría y "Tune: Tiro"
    private CellTarget lastTarget;
    private long lastTargetNs;
    private double lastLatencyMs = Double.NaN;
    private double last3dDistanceIn = Double.NaN;
    private final Map<Cell, Double> lastRowHeight = new EnumMap<>(Cell.class);
    private String status = "sin datos";
    private String lastReason = "";
    private boolean wrongPipeline = false;

    public CellVision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        this.belief = newBelief(alliance);
        limelight = hardwareMap.get(Limelight3A.class, Names.LIMELIGHT);
        limelight.setPollRateHz(VisionCfg.POLL_RATE_HZ);   // antes de start(): con el Limelight corriendo se ignora
        limelight.pipelineSwitch(VisionCfg.APRILTAG_PIPELINE);
        limelight.start();
    }

    /** Para la simulación: sin hardware, se alimenta con process(). */
    public CellVision(Alliance alliance) {
        this.alliance = alliance;
        this.belief = newBelief(alliance);
        this.limelight = null;
    }

    private static HiveBelief newBelief(Alliance alliance) {
        Cell start = Cell.valueOf(alliance == Alliance.RED ? VisionCfg.START_UP_RED : VisionCfg.START_UP_BLUE);
        return new HiveBelief(alliance, start, VisionCfg.UPDOWN_CONFIRM_FRAMES,
                VisionCfg.UPDOWN_OVERRIDE_FRAMES, VisionCfg.UPDOWN_STREAK_GAP_S);
    }

    /**
     * Llamar una vez por ciclo.
     * @return el objetivo SOLO si llegó una foto nueva con la CELL de arriba a la vista.
     */
    public CellTarget update(long nowNs) {
        if (limelight == null) return null;
        return process(limelight.getLatestResult(), nowNs);
    }

    /** Toda la lógica, sin hardware. Público para la simulación. */
    public CellTarget process(LLResult result, long nowNs) {
        if (result == null) return null;

        double ts = result.getTimestamp();
        if (ts == lastLlTimestamp) return null;          // la misma foto del Limelight
        lastLlTimestamp = ts;

        wrongPipeline = result.getPipelineIndex() != VisionCfg.APRILTAG_PIPELINE;
        if (!result.isValid()) {
            status = "no se ven tags";
            return null;
        }

        double ageMs = Math.max(0, result.getStaleness())
                + result.getCaptureLatency() + result.getTargetingLatency();
        lastLatencyMs = ageMs;
        if (ageMs > VisionCfg.MAX_LATENCY_MS) {
            status = "foto vieja";
            return null;
        }
        long captureNs = nowNs - (long) (ageMs * 1e6);

        // Separa los tags por CELL, solo los de nuestra alianza
        Map<Cell, List<TagSighting>> byCell = new EnumMap<>(Cell.class);
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            Cell cell = Cell.fromTagId(fr.getFiducialId());
            if (cell == null || cell.alliance != alliance) continue;
            List<TagSighting> list = byCell.get(cell);
            if (list == null) {
                list = new ArrayList<>();
                byCell.put(cell, list);
            }
            list.add(new TagSighting(
                    cell.memberIndex(fr.getFiducialId()),
                    fr.getTargetXDegreesNoCrosshair(),
                    VisionCfg.TY_SIGN * fr.getTargetYDegreesNoCrosshair(),
                    fr.getTargetArea(),
                    tagHeight3d(fr.getTargetPoseCameraSpace())));
        }
        if (byCell.isEmpty()) {
            status = "no se ven tags de " + alliance;
            return null;
        }

        // Medir la altura de cada fila visible y decidir cuál está arriba
        Map<Cell, Double> heights = new EnumMap<>(Cell.class);
        for (Map.Entry<Cell, List<TagSighting>> e : byCell.entrySet()) {
            double h = measureRowHeight(e.getValue());
            heights.put(e.getKey(), h);
            if (!Double.isNaN(h)) lastRowHeight.put(e.getKey(), h);
        }
        Cell claim = claimFromHeights(heights);
        belief.addEvidence(claim, nowNs, lastReason);

        Cell up = belief.upCell();
        if (claim != null && claim != up) {
            status = "esta foto dice que arriba está " + claim + ": no se tira";
            return null;
        }
        List<TagSighting> tags = byCell.get(up);
        if (tags == null) {
            status = "solo se ve la CELL de abajo";
            return null;
        }

        // Punto de tiro: centro de la abertura (2+ tags) o, con un tag, el centro estimado de la fila
        double heightAboveCam = VisionCfg.TAG_ROW_HEIGHT_UP_IN - VisionCfg.CAMERA_HEIGHT_IN;
        ClusterSolver.Solution sol = solver.solve(up, tags, nowNs);
        double aimF, aimL;
        boolean precise = false;
        double[] fit = tags.size() >= 2 ? HiveGeometry.rowFit(xs(tags), rays(tags), heightAboveCam) : null;
        if (fit != null) {
            double[] n = HiveGeometry.normalTowardCamera(fit);
            aimF = fit[0] + VisionCfg.APERTURE_OUT_IN * n[0];
            aimL = fit[1] + VisionCfg.APERTURE_OUT_IN * n[1];
            precise = true;
        } else {
            double[] hit = HiveGeometry.hitAtHeight(
                    HiveGeometry.ray(sol.txDeg, sol.tyDeg, VisionCfg.CAMERA_PITCH_DEG), heightAboveCam);
            if (hit == null) {
                status = "la fila quedó bajo el horizonte de la cámara";
                return null;
            }
            aimF = hit[0];
            aimL = hit[1];
        }

        last3dDistanceIn = mean3dDistance(result, up);
        double camAz = Math.toDegrees(Math.atan2(aimL, aimF));
        double[] axis = HiveGeometry.fromCameraToTurretAxis(camAz, Math.hypot(aimF, aimL),
                VisionCfg.CAMERA_BEHIND_AXIS_IN);

        status = precise ? "ok" : "un solo tag: no se puede fijar";
        lastTarget = new CellTarget(up, axis[0] + VisionCfg.AIM_TRIM_DEG, axis[1], tags.size(),
                sol.centered, precise, captureNs);
        lastTargetNs = nowNs;
        return lastTarget;
    }

    /** Altura de la fila (in del piso) por separación de tags; si no hay 2, por la pose 3D; si no, NaN. */
    private static double measureRowHeight(List<TagSighting> tags) {
        if (tags.size() >= 2) {
            double dh = HiveGeometry.rowHeightAboveCam(xs(tags), rays(tags));
            if (!Double.isNaN(dh)) return VisionCfg.CAMERA_HEIGHT_IN + dh;
        }
        if (VisionCfg.USE_3D_FOR_UPDOWN) {
            double sum = 0;
            int n = 0;
            for (TagSighting t : tags) {
                if (!Double.isNaN(t.height3dIn)) {
                    sum += t.height3dIn;
                    n++;
                }
            }
            if (n > 0) return sum / n;
        }
        return Double.NaN;
    }

    /** Cuál CELL está arriba según las alturas medidas en esta foto, o null si no alcanza. */
    private Cell claimFromHeights(Map<Cell, Double> heights) {
        Cell[] both = Cell.ofAlliance(alliance);
        double ha = heights.containsKey(both[0]) ? heights.get(both[0]) : Double.NaN;
        double hb = heights.containsKey(both[1]) ? heights.get(both[1]) : Double.NaN;
        double mid = 0.5 * (VisionCfg.TAG_ROW_HEIGHT_UP_IN + VisionCfg.TAG_ROW_HEIGHT_DOWN_IN);
        double m = VisionCfg.UPDOWN_HEIGHT_MARGIN_IN;

        if (!Double.isNaN(ha) && !Double.isNaN(hb)) {
            if (Math.abs(ha - hb) < 2 * m) return null;
            lastReason = String.format("dos filas: %.0f y %.0f in", Math.max(ha, hb), Math.min(ha, hb));
            return ha > hb ? both[0] : both[1];
        }
        for (Cell c : both) {
            double h = c == both[0] ? ha : hb;
            if (Double.isNaN(h)) continue;
            if (h > mid + m) {
                lastReason = String.format("fila a %.0f in", h);
                return c;
            }
            if (h < mid - m) {
                lastReason = String.format("fila a %.0f in", h);
                return c.partner();
            }
        }
        return null;
    }

    private static double[] xs(List<TagSighting> tags) {
        double[] x = new double[tags.size()];
        for (int i = 0; i < x.length; i++) x[i] = Cell.TAG_X_IN[tags.get(i).memberIndex];
        return x;
    }

    private static HiveGeometry.Ray[] rays(List<TagSighting> tags) {
        HiveGeometry.Ray[] r = new HiveGeometry.Ray[tags.size()];
        for (int i = 0; i < r.length; i++) {
            r[i] = HiveGeometry.ray(tags.get(i).txDeg, tags.get(i).tyDeg, VisionCfg.CAMERA_PITCH_DEG);
        }
        return r;
    }

    /** Altura de un tag por la pose 3D del Limelight, o NaN si "Full 3D" está apagado. */
    private static double tagHeight3d(Pose3D pose) {
        if (pose == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return Double.NaN;
        return HiveGeometry.heightFromCameraSpace(p.y, p.z, VisionCfg.CAMERA_HEIGHT_IN, VisionCfg.CAMERA_PITCH_DEG);
    }

    /** Distancia horizontal promedio de la cámara a los tags de una CELL por la pose 3D (solo telemetría). */
    private static double mean3dDistance(LLResult result, Cell cell) {
        double sum = 0;
        int n = 0;
        double p = Math.toRadians(VisionCfg.CAMERA_PITCH_DEG);
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            if (Cell.fromTagId(fr.getFiducialId()) != cell) continue;
            Pose3D pose = fr.getTargetPoseCameraSpace();
            if (pose == null) continue;
            Position pos = pose.getPosition().toUnit(DistanceUnit.INCH);
            if (pos.x == 0 && pos.y == 0 && pos.z == 0) continue;
            double forward = pos.z * Math.cos(p) + pos.y * Math.sin(p);
            sum += Math.hypot(pos.x, forward);
            n++;
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    // ---------- piloto ----------

    /** El artillero avisa que el HIVE se volteó (o que la visión eligió mal). */
    public void flipHive() {
        belief.flip();
        solver.reset();
    }

    public Cell upCell() {
        return belief.upCell();
    }

    // ---------- lectura ----------

    /** Último objetivo visto, aunque no sea de este ciclo. Puede ser null. */
    public CellTarget getLastTarget() { return lastTarget; }

    public double secondsSinceTarget(long nowNs) {
        return lastTarget == null ? Double.POSITIVE_INFINITY : (nowNs - lastTargetNs) / 1e9;
    }

    public double get3dDistanceIn() { return last3dDistanceIn; }
    public String getStatus() { return status; }

    /** Última altura medida de la fila de una CELL (in), NaN si nunca se midió. */
    public double getRowHeightIn(Cell cell) {
        Double h = lastRowHeight.get(cell);
        return h == null ? Double.NaN : h;
    }

    public void addTelemetry(Telemetry telemetry, long nowNs) {
        if (limelight != null && !limelight.isConnected()) {
            telemetry.addLine("!! LIMELIGHT DESCONECTADO");
            return;
        }
        if (wrongPipeline) telemetry.addLine("!! El Limelight no está en el pipeline de AprilTags (cambiarlo en su página)");
        Cell[] both = Cell.ofAlliance(alliance);
        telemetry.addData("HIVE", "arriba: %s (%s)", belief.upCell(), belief.reason());
        telemetry.addData("  filas medidas", "%s %.1f in · %s %.1f in", both[0], getRowHeightIn(both[0]),
                both[1], getRowHeightIn(both[1]));
        if (lastTarget == null || secondsSinceTarget(nowNs) > 0.5) {
            telemetry.addData("Visión", status);
            return;
        }
        telemetry.addData("Visión", "%s  tags %d/4  %s", lastTarget.cell, lastTarget.tagCount, status);
        telemetry.addData("  rumbo / dist", "%.1f° / %.1f in   retraso %.0f ms",
                lastTarget.bearingDeg, lastTarget.distanceIn, lastLatencyMs);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
